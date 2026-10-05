// Diggable regolith for the Lunabotics arena.
//
// gz-sim terrain is a rigid heightmap: nothing can dig into it, and it cannot be
// edited once loaded. This world system fakes excavation on top of that:
//
//   - It keeps its own copy of the bed's height grid, starting from Moon_DEM's
//     heightmap.
//   - The bucket does not collide with the bed (collide_bitmask: bucket bit 1,
//     bed bit 0), so lowering it puts the tray floor INTO the bed. Every grid
//     vertex the floor passes below is cut down to the floor, and that volume
//     goes into the bucket, until it is full.
//   - The load is applied to the jaw as its weight, and the cut as a drag force
//     against the bucket's motion, proportional to the width x depth being cut.
//     Both are added every physics step, so the drive feels them.
//   - Opening the jaw (the real rover bottom-dumps) or turning the bucket over
//     pours the load out under the bucket. It lands as a pile and slumps to the
//     angle of repose, so dumps build a berm.
//   - Whenever the grid has changed, at most every rebuild_period_s, the grid is
//     written out as a 16-bit PNG and the terrain model is REPLACED: the new one
//     is created, then the old one removed. The lidar, the cameras and the
//     wheels all see trenches and berms after that.
//
// Also adds rolling resistance (crr x weight against the rover's horizontal
// velocity) while the rover is on the bed, for wheels in loose regolith.
//
// Status: gz topic /regolith/status (gz.msgs.StringMsg, JSON) at 2 Hz and the
// load on /regolith/bucket_kg (gz.msgs.Double), bridged to ROS.

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <zlib.h>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Root.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace perseus_simulation
{
namespace fs = std::filesystem;
using gz::math::Pose3d;
using gz::math::Vector3d;
using gz::sim::Entity;
using gz::sim::kNullEntity;

class RegolithDig : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
public:
    void Configure(const Entity& world, const std::shared_ptr<const sdf::Element>& sdf,
                   gz::sim::EntityComponentManager& /*ecm*/, gz::sim::EventManager& eventMgr) override
    {
        _world = world;
        _eventMgr = &eventMgr;
        auto get = [&](const char* name, auto def) {
            return sdf->Get<decltype(def)>(name, def).first;
        };
        _robotName = get("robot_model", std::string("perseus"));
        _jawLink = get("jaw_link", std::string("bucket_jaw"));
        _bodyLink = get("body_link", std::string("bucket_body"));
        _baseLink = get("base_link", std::string("base_link"));
        _terrainName = get("terrain_model", std::string("moon"));
        _terrainDir = get("terrain_dir", std::string("Moon_DEM"));
        _terrainImage = get("terrain_image", std::string("dem/regolith_heightmap.png"));
        _size = get("terrain_size", Vector3d(9.44, 8.40, 0.30));
        _origin = get("terrain_pose", Vector3d(0, 4.05, -0.212));
        _floorCentre = get("floor_centre", Vector3d(0.04855, 0.0, -0.20294));
        _floorHalf = get("floor_half_size", gz::math::Vector2d(0.085, 0.23));
        _capacity = get("capacity_m3", 0.012);
        _density = get("bulk_density", 1500.0);
        _cutPressure = get("cut_pressure_pa", 8000.0);
        _fullPressure = get("bulldoze_pressure_pa", 20000.0);
        _dumpJaw = get("dump_jaw_rad", 0.25);
        _dumpRate = get("dump_rate_kg_s", 10.0);
        _repose = std::tan(get("repose_deg", 35.0) * M_PI / 180.0);
        _crr = get("rolling_resistance", 0.05);
        _rebuildPeriod = get("rebuild_period_s", 2.0);
        _newFloor = get("dig_floor_z", -0.512);  // lowest diggable / PNG zero
        _newRange = get("height_range", 1.0);   // PNG full scale

        if (!LoadBaseGrid())
            return;
        char tmpl[] = "/tmp/regolith_XXXXXX";
        if (const char* d = mkdtemp(tmpl))
            _tmpDir = d;
        _statusPub = _node.Advertise<gz::msgs::StringMsg>("/regolith/status");
        _loadPub = _node.Advertise<gz::msgs::Double>("/regolith/bucket_kg");
        _ready = true;
        gzmsg << "[RegolithDig] " << _nx << "x" << _ny << " grid from " << _terrainPath
              << ", capacity " << _capacity * _density << " kg, rebuilds in " << _tmpDir << "\n";
    }

    void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override
    {
        if (!_ready || info.paused)
            return;
        const double t = std::chrono::duration<double>(info.simTime).count();
        const double dt = std::chrono::duration<double>(info.dt).count();
        if (!FindEntities(ecm))
            return;

        const Pose3d jaw = gz::sim::worldPose(_jaw, ecm);
        const Pose3d body = gz::sim::worldPose(_body, ecm);
        const Pose3d floor = jaw * Pose3d(_floorCentre, gz::math::Quaterniond::Identity);
        const Vector3d n = floor.Rot().RotateVector(Vector3d::UnitZ);

        // Jaw opening = rotation of the jaw about the body's y axis.
        const gz::math::Quaterniond rel = body.Rot().Inverse() * jaw.Rot();
        const double jawOpen = std::abs(rel.Euler().Y());

        // ---- cut: at 50 Hz, the grid changes slowly next to a 1 kHz step
        if (t - _lastCut >= 0.02) {
            _lastCut = t;
            _depth = Cut(floor, n);
        }

        // ---- dump
        if (_loadKg > 0 && (jawOpen > _dumpJaw || n.Z() < 0.0)) {
            const double kg = std::min(_loadKg, _dumpRate * dt);
            _pour += kg;
            _loadKg -= kg;
            if (_pour >= 0.25 || _loadKg <= 1e-9) {  // deposit in 0.25 kg lumps
                Deposit(floor, _pour / _density);
                _pour = 0;
            }
        }

        // ---- forces on the bucket
        gz::sim::Link jawLink(_jaw);
        const auto v = jawLink.WorldLinearVelocity(ecm).value_or(Vector3d::Zero);
        Vector3d force(0, 0, -9.81 * _loadKg);
        const Vector3d vh(v.X(), v.Y(), 0);
        if (_depth > 0 && vh.Length() > 1e-4) {
            const double p = _loadKg >= _capacity * _density - 1e-6 ? _fullPressure : _cutPressure;
            const double mag = p * 2 * _floorHalf.Y() * _depth * std::min(1.0, vh.Length() / 0.05);
            force -= vh.Normalized() * mag;
            _lastDrag = mag;
        } else {
            _lastDrag = 0;
        }
        if (force.Length() > 0)
            jawLink.AddWorldForce(ecm, force, _floorCentre);

        // ---- rolling resistance on the bed
        gz::sim::Link base(_base);
        const Pose3d bp = gz::sim::worldPose(_base, ecm);
        const auto bv = base.WorldLinearVelocity(ecm).value_or(Vector3d::Zero);
        const Vector3d bvh(bv.X(), bv.Y(), 0);
        if (_crr > 0 && OnBed(bp.Pos().X(), bp.Pos().Y()) && bvh.Length() > 1e-4) {
            const double mag = _crr * _robotMass * 9.81 * std::min(1.0, bvh.Length() / 0.05);
            base.AddWorldForce(ecm, -bvh.Normalized() * mag);
        }

        // ---- terrain replacement
        if (_dirty && t - _lastRebuild >= _rebuildPeriod)
            Rebuild(ecm, t);

        if (t - _lastStatus >= 0.5) {
            _lastStatus = t;
            PublishStatus(t, jawOpen);
        }
    }

private:
    // Grid vertex (i, j): i along +X, j along +Y. PNG row 0 is the +Y (north)
    // edge - gz flips image heightmaps so the image reads like a map.
    double X(int i) const { return _origin.X() - _size.X() / 2 + i * _dx; }
    double Y(int j) const { return _origin.Y() - _size.Y() / 2 + j * _dy; }
    double& H(int i, int j) { return _h[static_cast<size_t>(j) * _nx + i]; }
    bool OnBed(double x, double y) const
    {
        return std::abs(x - _origin.X()) < _size.X() / 2 && std::abs(y - _origin.Y()) < _size.Y() / 2;
    }

    bool LoadBaseGrid()
    {
        for (const char* var : {"GZ_SIM_RESOURCE_PATH"}) {
            const char* env = std::getenv(var);
            std::stringstream ss(env ? env : "");
            std::string dir;
            while (std::getline(ss, dir, ':')) {
                if (!dir.empty() && fs::exists(fs::path(dir) / _terrainDir / _terrainImage)) {
                    _modelDir = (fs::path(dir) / _terrainDir).string();
                    break;
                }
            }
        }
        if (_modelDir.empty()) {
            gzerr << "[RegolithDig] " << _terrainDir << "/" << _terrainImage
                  << " not found on GZ_SIM_RESOURCE_PATH; digging disabled\n";
            return false;
        }
        _terrainPath = _modelDir + "/" + _terrainImage;
        gz::common::Image img;
        if (img.Load(_terrainPath) != 0 || img.Width() != img.Height()) {
            gzerr << "[RegolithDig] cannot load square heightmap " << _terrainPath << "\n";
            return false;
        }
        _nx = _ny = static_cast<int>(img.Width());
        _dx = _size.X() / (_nx - 1);
        _dy = _size.Y() / (_ny - 1);
        const bool sixteen = img.BPP() >= 16 && img.PixelFormat() == gz::common::Image::L_INT16;
        const auto data = img.RGBData();  // 8-bit per channel either way
        const auto data16 = sixteen ? img.Data() : std::vector<unsigned char>();
        _h.assign(static_cast<size_t>(_nx) * _ny, 0.0);
        for (int r = 0; r < _ny; ++r) {
            for (int c = 0; c < _nx; ++c) {
                double frac;
                if (sixteen) {
                    const size_t k = (static_cast<size_t>(r) * _nx + c) * 2;
                    frac = (data16[k] | (data16[k + 1] << 8)) / 65535.0;
                } else {
                    frac = data[(static_cast<size_t>(r) * _nx + c) * 3] / 255.0;
                }
                H(c, _ny - 1 - r) = _origin.Z() + frac * _size.Z();
            }
        }
        return true;
    }

    bool FindEntities(gz::sim::EntityComponentManager& ecm)
    {
        if (_robot != kNullEntity && ecm.HasEntity(_robot))
            return _jaw != kNullEntity && _body != kNullEntity && _base != kNullEntity;
        _robot = gz::sim::World(_world).ModelByName(ecm, _robotName);
        if (_robot == kNullEntity)
            return false;
        gz::sim::Model model(_robot);
        _jaw = model.LinkByName(ecm, _jawLink);
        _body = model.LinkByName(ecm, _bodyLink);
        _base = model.LinkByName(ecm, _baseLink);
        if (_jaw == kNullEntity || _body == kNullEntity || _base == kNullEntity) {
            if (!_warnedLinks)
                gzwarn << "[RegolithDig] " << _robotName << " has no " << _jawLink << "/" << _bodyLink
                       << "/" << _baseLink << " (payload:=none?); digging idle\n";
            _warnedLinks = true;
            return false;
        }
        gz::sim::Link(_jaw).EnableVelocityChecks(ecm, true);
        gz::sim::Link(_base).EnableVelocityChecks(ecm, true);
        _robotMass = 0;
        for (const Entity l : model.Links(ecm))
            if (auto in = ecm.Component<gz::sim::components::Inertial>(l))
                _robotMass += in->Data().MassMatrix().Mass();
        gzmsg << "[RegolithDig] tracking " << _robotName << " (" << _robotMass << " kg)\n";
        return true;
    }

    // Cuts every grid vertex inside the tray floor's footprint down to the floor,
    // while there is room in the bucket. Returns the deepest penetration.
    double Cut(const Pose3d& floor, const Vector3d& n)
    {
        if (n.Z() < 0.3)  // floor steeper than ~73 deg: it is not scooping
            return 0;
        const double reach = std::hypot(_floorHalf.X(), _floorHalf.Y());
        const Vector3d c = floor.Pos();
        const int i0 = std::max(0, static_cast<int>(std::floor((c.X() - reach - X(0)) / _dx)));
        const int i1 = std::min(_nx - 1, static_cast<int>(std::ceil((c.X() + reach - X(0)) / _dx)));
        const int j0 = std::max(0, static_cast<int>(std::floor((c.Y() - reach - Y(0)) / _dy)));
        const int j1 = std::min(_ny - 1, static_cast<int>(std::ceil((c.Y() + reach - Y(0)) / _dy)));
        const double cell = _dx * _dy;
        const double fullKg = _capacity * _density;
        double depth = 0;
        bool cut = false;
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                const double z = c.Z() - (n.X() * (X(i) - c.X()) + n.Y() * (Y(j) - c.Y())) / n.Z();
                const Vector3d local = floor.Rot().RotateVectorReverse(Vector3d(X(i), Y(j), z) - c);
                if (std::abs(local.X()) > _floorHalf.X() || std::abs(local.Y()) > _floorHalf.Y())
                    continue;
                double& h = H(i, j);
                const double zc = std::max(z, _newFloor + 0.01);
                if (zc >= h)
                    continue;
                depth = std::max(depth, h - zc);
                const double roomKg = fullKg - _loadKg;
                if (roomKg <= 0)
                    continue;
                const double dh = std::min(h - zc, roomKg / (_density * cell));
                h -= dh;
                _loadKg += dh * cell * _density;
                _dugKg += dh * cell * _density;
                cut = true;
            }
        }
        if (cut) {
            Relax(c.X(), c.Y(), 0.6, 4);  // trench walls slump a little
            _dirty = true;
        }
        return depth;
    }

    // Pours volume v (m3) out of the tray: spread evenly over the tray floor's
    // footprint on the ground under it, then left to slump to the angle of
    // repose. (A point source made one-cell spikes, which the renderer and the
    // physics heightmap interpolate differently.)
    void Deposit(const Pose3d& floor, double v)
    {
        _dumpedKg += v * _density;
        const double x = floor.Pos().X(), y = floor.Pos().Y();
        if (!OnBed(x, y))
            return;  // poured over the wall: gone
        const double yaw = floor.Rot().Euler().Z();
        const double c = std::cos(yaw), s = std::sin(yaw);
        const double hx = std::max(_floorHalf.X(), _dx), hy = std::max(_floorHalf.Y(), _dy);
        const double reach = std::hypot(hx, hy);
        std::vector<size_t> cells;
        for (int j = std::max(0, static_cast<int>((y - reach - Y(0)) / _dy));
             j <= std::min(_ny - 1, static_cast<int>((y + reach - Y(0)) / _dy) + 1); ++j)
            for (int i = std::max(0, static_cast<int>((x - reach - X(0)) / _dx));
                 i <= std::min(_nx - 1, static_cast<int>((x + reach - X(0)) / _dx) + 1); ++i) {
                const double lx = c * (X(i) - x) + s * (Y(j) - y);
                const double ly = -s * (X(i) - x) + c * (Y(j) - y);
                if (std::abs(lx) <= hx && std::abs(ly) <= hy)
                    cells.push_back(static_cast<size_t>(j) * _nx + i);
            }
        if (cells.empty())
            return;
        for (const size_t k : cells)
            _h[k] += v / (_dx * _dy * cells.size());
        if (std::abs(x - 3.56) < 0.45 && std::abs(y - 6.40) < 1.10)
            _bermKg += v * _density;  // landed in the target berm area
        Relax(x, y, 1.2, 30);
        _dirty = true;
    }

    // Moves material downhill wherever a slope between neighbours exceeds the
    // angle of repose, inside a square window of half-width r. Conserves volume.
    void Relax(double x, double y, double r, int iterations)
    {
        const int i0 = std::max(1, static_cast<int>((x - r - X(0)) / _dx));
        const int i1 = std::min(_nx - 2, static_cast<int>((x + r - X(0)) / _dx));
        const int j0 = std::max(1, static_cast<int>((y - r - Y(0)) / _dy));
        const int j1 = std::min(_ny - 2, static_cast<int>((y + r - Y(0)) / _dy));
        const double sx = _repose * _dx, sy = _repose * _dy;
        for (int it = 0; it < iterations; ++it) {
            bool moved = false;
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i) {
                    const std::pair<int, int> nb[4] = {{i + 1, j}, {i - 1, j}, {i, j + 1}, {i, j - 1}};
                    for (const auto& [a, b] : nb) {
                        const double lim = (a != i) ? sx : sy;
                        const double diff = H(i, j) - H(a, b) - lim;
                        if (diff > 1e-4) {
                            const double m = diff / 5;
                            H(i, j) -= m;
                            H(a, b) += m;
                            moved = true;
                        }
                    }
                }
            if (!moved)
                break;
        }
    }

    std::string TerrainSdf(const std::string& name, const std::string& png) const
    {
        const double sz = _newRange;
        std::ostringstream s;
        s << "<?xml version='1.0'?><sdf version='1.9'><model name='" << name << "'>"
          << "<static>true</static><pose>" << _origin.X() << " " << _origin.Y() << " " << _newFloor
          << " 0 0 0</pose><link name='link'>"
          << "<collision name='collision'><geometry><heightmap><uri>" << png << "</uri><size>"
          << _size.X() << " " << _size.Y() << " " << sz << "</size><pos>0 0 0</pos></heightmap></geometry>"
          << "<surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction>"
          << "<contact><collide_bitmask>0x01</collide_bitmask></contact></surface></collision>"
          << "<visual name='visual'><geometry><heightmap><use_terrain_paging>false</use_terrain_paging>"
          << "<texture><diffuse>" << _modelDir << "/materials/textures/regolith_diffuse.png</diffuse>"
          << "<normal>" << _modelDir << "/materials/textures/moon_normal.png</normal><size>2</size></texture>"
          << "<uri>" << png << "</uri><size>" << _size.X() << " " << _size.Y() << " " << sz << "</size>"
          << "<pos>" << _origin.X() << " " << _origin.Y() << " " << _newFloor << "</pos>"
          << "<sampling>2</sampling></heightmap></geometry></visual></link></model></sdf>";
        return s.str();
    }

    void Rebuild(gz::sim::EntityComponentManager& ecm, double t)
    {
        _lastRebuild = t;
        _dirty = false;
        // 16-bit, absolute: z = dig_floor_z + pixel / 65535 * height_range. Row 0 = north.
        std::vector<uint16_t> px(static_cast<size_t>(_nx) * _ny);
        for (int r = 0; r < _ny; ++r)
            for (int c = 0; c < _nx; ++c) {
                const double f = std::clamp((H(c, _ny - 1 - r) - _newFloor) / _newRange, 0.0, 1.0);
                px[static_cast<size_t>(r) * _nx + c] = static_cast<uint16_t>(std::lround(f * 65535));
            }
        // Pin the SW corner to 0 and the NE corner to full scale. The ogre2
        // renderer stretches the image's own min..max over the heightmap's z
        // size, while physics uses the pixel values as they are; with both
        // extremes always present the two agree. Without this the rendered bed
        // floated ~0.15 m above the collision bed. Both corners are under walls.
        px[static_cast<size_t>(_ny - 1) * _nx] = 0;
        px[static_cast<size_t>(_nx - 1)] = 65535;
        const std::string name = "regolith_" + std::to_string(++_generation);
        const std::string png = _tmpDir + "/" + name + ".png";
        if (!WritePng16(png, px, _nx, _ny)) {
            gzerr << "[RegolithDig] could not write " << png << "\n";
            return;
        }

        sdf::Root root;
        const auto errors = root.LoadSdfString(TerrainSdf(name, png));
        if (!errors.empty() || root.Model() == nullptr) {
            gzerr << "[RegolithDig] terrain SDF rejected: " << (errors.empty() ? "" : errors[0].Message()) << "\n";
            return;
        }
        gz::sim::SdfEntityCreator creator(ecm, *_eventMgr);
        const Entity fresh = creator.CreateEntities(root.Model());
        creator.SetParent(fresh, _world);

        const Entity old = gz::sim::World(_world).ModelByName(ecm, _currentTerrain.empty() ? _terrainName : _currentTerrain);
        if (old != kNullEntity)
            creator.RequestRemoveEntity(old, true);
        _currentTerrain = name;
        // The renderers load the PNG after the entity arrives; keep a few back.
        if (_generation > 3)
            fs::remove(_tmpDir + "/regolith_" + std::to_string(_generation - 3) + ".png");
    }

    // gz::common::Image::SavePNG writes an empty file for L_INT16, so this is a
    // minimal 16-bit greyscale PNG encoder (samples are big-endian in PNG).
    static bool WritePng16(const std::string& path, const std::vector<uint16_t>& px, int w, int h)
    {
        std::vector<unsigned char> raw;
        raw.reserve(static_cast<size_t>(h) * (1 + 2 * w));
        for (int r = 0; r < h; ++r) {
            raw.push_back(0);  // filter: none
            for (int c = 0; c < w; ++c) {
                const uint16_t v = px[static_cast<size_t>(r) * w + c];
                raw.push_back(static_cast<unsigned char>(v >> 8));
                raw.push_back(static_cast<unsigned char>(v & 0xff));
            }
        }
        uLongf zlen = compressBound(raw.size());
        std::vector<unsigned char> z(zlen);
        if (compress2(z.data(), &zlen, raw.data(), raw.size(), 6) != Z_OK)
            return false;
        z.resize(zlen);

        std::ofstream out(path, std::ios::binary);
        auto be32 = [&](uint32_t v) {
            const unsigned char b[4] = {static_cast<unsigned char>(v >> 24), static_cast<unsigned char>(v >> 16),
                                        static_cast<unsigned char>(v >> 8), static_cast<unsigned char>(v)};
            out.write(reinterpret_cast<const char*>(b), 4);
        };
        auto chunk = [&](const char* type, const std::vector<unsigned char>& data) {
            be32(static_cast<uint32_t>(data.size()));
            out.write(type, 4);
            out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
            uLong crc = crc32(0, reinterpret_cast<const Bytef*>(type), 4);
            if (!data.empty())  // crc32() with a null buffer returns the seed, 0
                crc = crc32(crc, data.data(), static_cast<uInt>(data.size()));
            be32(static_cast<uint32_t>(crc));
        };
        const unsigned char sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
        out.write(reinterpret_cast<const char*>(sig), 8);
        std::vector<unsigned char> ihdr = {
            static_cast<unsigned char>(w >> 24), static_cast<unsigned char>(w >> 16),
            static_cast<unsigned char>(w >> 8),  static_cast<unsigned char>(w),
            static_cast<unsigned char>(h >> 24), static_cast<unsigned char>(h >> 16),
            static_cast<unsigned char>(h >> 8),  static_cast<unsigned char>(h),
            16, 0, 0, 0, 0};  // bit depth 16, greyscale, deflate, no filter, no interlace
        chunk("IHDR", ihdr);
        chunk("IDAT", z);
        chunk("IEND", {});
        return static_cast<bool>(out);
    }

    void PublishStatus(double t, double jawOpen)
    {
        gz::msgs::Double load;
        load.set_data(_loadKg);
        _loadPub.Publish(load);
        std::ostringstream s;
        s.precision(3);
        s << std::fixed << "{\"t\":" << t << ",\"bucket_kg\":" << _loadKg << ",\"dug_kg\":" << _dugKg
          << ",\"dumped_kg\":" << _dumpedKg << ",\"berm_kg\":" << _bermKg << ",\"cut_depth_m\":" << _depth
          << ",\"drag_n\":" << _lastDrag << ",\"jaw_open_rad\":" << jawOpen
          << ",\"terrain\":\"" << (_currentTerrain.empty() ? _terrainName : _currentTerrain) << "\"}";
        gz::msgs::StringMsg msg;
        msg.set_data(s.str());
        _statusPub.Publish(msg);
    }

    // config
    std::string _robotName, _jawLink, _bodyLink, _baseLink, _terrainName, _terrainDir, _terrainImage;
    Vector3d _size, _origin, _floorCentre;
    gz::math::Vector2d _floorHalf;
    double _capacity{}, _density{}, _cutPressure{}, _fullPressure{}, _dumpJaw{}, _dumpRate{};
    double _repose{}, _crr{}, _rebuildPeriod{}, _newFloor{}, _newRange{};
    // state
    bool _ready{false}, _dirty{false}, _warnedLinks{false};
    Entity _world{kNullEntity}, _robot{kNullEntity}, _jaw{kNullEntity}, _body{kNullEntity}, _base{kNullEntity};
    gz::sim::EventManager* _eventMgr{nullptr};
    std::string _modelDir, _terrainPath, _tmpDir, _currentTerrain;
    int _nx{0}, _ny{0}, _generation{0};
    double _dx{0}, _dy{0};
    std::vector<double> _h;
    double _loadKg{0}, _pour{0}, _dugKg{0}, _dumpedKg{0}, _bermKg{0}, _depth{0}, _lastDrag{0}, _robotMass{0};
    double _lastCut{-1}, _lastRebuild{-1e9}, _lastStatus{-1};
    gz::transport::Node _node;
    gz::transport::Node::Publisher _statusPub, _loadPub;
};
}  // namespace perseus_simulation

GZ_ADD_PLUGIN(perseus_simulation::RegolithDig, gz::sim::System,
              perseus_simulation::RegolithDig::ISystemConfigure,
              perseus_simulation::RegolithDig::ISystemPreUpdate)
