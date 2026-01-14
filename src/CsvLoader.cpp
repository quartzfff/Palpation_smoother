#include "CsvLoader.hpp"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <stdexcept>
#include <cctype>

static std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  bool in_quotes = false;
  for (char c : line) {
    if (c == '"') { in_quotes = !in_quotes; continue; }
    if (c == ',' && !in_quotes) { out.push_back(cur); cur.clear(); }
    else cur.push_back(c);
  }
  out.push_back(cur);
  return out;
}

static double to_double(const std::string& s) {
  // std::stod tolerates leading/trailing spaces
  return std::stod(s);
}

// Quaternion (w,x,y,z) to rotation matrix
static Eigen::Matrix3d quat_to_R(double w, double x, double y, double z) {
  // normalize for safety
  double n = std::sqrt(w*w + x*x + y*y + z*z);
  if (n <= 1e-12) return Eigen::Matrix3d::Identity();
  w /= n; x /= n; y /= n; z /= n;

  Eigen::Matrix3d R;
  R(0,0) = 1 - 2*(y*y + z*z);
  R(0,1) = 2*(x*y - z*w);
  R(0,2) = 2*(x*z + y*w);

  R(1,0) = 2*(x*y + z*w);
  R(1,1) = 1 - 2*(x*x + z*z);
  R(1,2) = 2*(y*z - x*w);

  R(2,0) = 2*(x*z - y*w);
  R(2,1) = 2*(y*z + x*w);
  R(2,2) = 1 - 2*(x*x + y*y);
  return R;
}

std::vector<MeasRow> load_meas_csv(
    const std::string& csv_path,
    const Eigen::Matrix3d& R_calib,
    const Eigen::Vector3d& t_calib,
    const Eigen::Vector3d& p_offset_base,
    int stride)
{
  if (stride < 1) stride = 1;

  std::ifstream fin(csv_path);
  if (!fin) throw std::runtime_error("Failed to open CSV: " + csv_path);

  std::string header;
  if (!std::getline(fin, header)) throw std::runtime_error("Empty CSV: " + csv_path);

  auto cols = split_csv_line(header);
  std::unordered_map<std::string, int> idx;
  idx.reserve(cols.size());
  for (int i = 0; i < (int)cols.size(); ++i) idx[cols[i]] = i;

  auto need = [&](const std::string& name) -> int {
    auto it = idx.find(name);
    if (it == idx.end())
      throw std::runtime_error("CSV missing column: " + name);
    return it->second;
  };

  // joints
  int i_j0 = need("joint_0");
  int i_j1 = need("joint_1");
  int i_j2 = need("joint_2");
  int i_j3 = need("joint_3");

  // ndi position + quaternion
  int i_px = need("ndi_px");
  int i_py = need("ndi_py");
  int i_pz = need("ndi_pz");
  int i_ox = need("ndi_ox");
  int i_oy = need("ndi_oy");
  int i_oz = need("ndi_oz");
  int i_ow = need("ndi_ow");

  std::vector<MeasRow> out;
  out.reserve(20000);

  std::string line;
  int row = 0;
  while (std::getline(fin, line)) {
    if (line.empty()) continue;
    if ((row % stride) != 0) { row++; continue; }

    auto f = split_csv_line(line);
    if ((int)f.size() < (int)cols.size()) { row++; continue; }

    // ---- joints mapping (match MATLAB) ----
    JointState q;
    q.d1     = to_double(f[i_j3]) * 1000.0;   // outer translation: m -> mm
    q.d2     = to_double(f[i_j2]) * 1000.0;   // inner translation: m -> mm
    q.theta1 = to_double(f[i_j1]);            // outer rotation: rad
    q.theta2 = to_double(f[i_j0]);            // inner rotation: rad

    // ---- NDI pose ----
    Eigen::Vector3d p_ndi(to_double(f[i_px]), to_double(f[i_py]), to_double(f[i_pz])); // mm
    double ox = to_double(f[i_ox]);
    double oy = to_double(f[i_oy]);
    double oz = to_double(f[i_oz]);
    double ow = to_double(f[i_ow]);

    Eigen::Matrix3d R_ndi = quat_to_R(ow, ox, oy, oz);

    // ---- transform to base (match MATLAB) ----
    Eigen::Vector3d p_base = R_calib * p_ndi + t_calib + p_offset_base;

    Eigen::Vector3d t_base = R_calib * (R_ndi * Eigen::Vector3d(0,0,1));
    double tn = t_base.norm();
    if (tn > 1e-12) t_base /= tn;

    out.push_back({q, p_base, t_base});
    row++;
  }

  return out;
}
