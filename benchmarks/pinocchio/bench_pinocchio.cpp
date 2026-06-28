#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joints.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial.hpp>

#include <Eigen/Core>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
struct Config
{
  long long iterations = 50000;
  long long warmup = 5000;
  bool include_init = true;
  bool correctness = false;
};

struct MeasureResult
{
  long long total_ns;
  double checksum;
};

void do_not_optimize(const double & value)
{
#if defined(__GNUC__) || defined(__clang__)
  asm volatile("" : : "g"(&value) : "memory");
#else
  (void)value;
#endif
}

Config parse_args(int argc, char ** argv)
{
  Config config;
  for(int i = 1; i < argc; ++i)
  {
    const std::string arg(argv[i]);
    if(arg == "--iterations")
    {
      if(++i >= argc)
        throw std::runtime_error("--iterations requires a value");
      config.iterations = std::atoll(argv[i]);
    }
    else if(arg == "--warmup")
    {
      if(++i >= argc)
        throw std::runtime_error("--warmup requires a value");
      config.warmup = std::atoll(argv[i]);
    }
    else if(arg == "--no-init")
    {
      config.include_init = false;
    }
    else if(arg == "--correctness")
    {
      config.correctness = true;
    }
    else if(arg == "--help" || arg == "-h")
    {
      std::cout << "Usage: pinocchio_bench [--iterations N] [--warmup N] [--no-init] [--correctness]\n";
      std::exit(0);
    }
    else
    {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  return config;
}

template<typename Operation>
MeasureResult measure(long long iterations, long long warmup, Operation && operation)
{
  double checksum = 0.0;
  for(long long i = 0; i < warmup; ++i)
  {
    checksum += operation();
    do_not_optimize(checksum);
  }

  const auto start = std::chrono::steady_clock::now();
  for(long long i = 0; i < iterations; ++i)
  {
    checksum += operation();
    do_not_optimize(checksum);
  }
  const auto end = std::chrono::steady_clock::now();

  const auto total_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  return {total_ns, checksum};
}

template<typename Operation>
void emit(
  const std::string & group,
  const std::string & model_name,
  const std::string & algorithm,
  int dof,
  const Config & config,
  Operation && operation)
{
  const MeasureResult result = measure(config.iterations, config.warmup, operation);
  const double ns_per_iter = static_cast<double>(result.total_ns) / config.iterations;
  std::cout << "pinocchio," << group << "," << model_name << "," << algorithm << "," << dof
            << "," << config.iterations << "," << result.total_ns << "," << std::fixed
            << std::setprecision(3) << ns_per_iter << "," << std::scientific
            << std::setprecision(12) << result.checksum << "\n";
}

Eigen::Matrix3d rz(double theta)
{
  const double sin = std::sin(theta);
  const double cos = std::cos(theta);
  Eigen::Matrix3d rotation;
  rotation << cos, -sin, 0.0, sin, cos, 0.0, 0.0, 0.0, 1.0;
  return rotation;
}

pinocchio::Inertia rigid_inertia(double mass, const Eigen::Vector3d & com)
{
  return pinocchio::Inertia(mass, com, Eigen::Matrix3d::Identity() * 0.01);
}

pinocchio::Model serial_revolute_model(int dof)
{
  pinocchio::Model model;
  model.gravity.linear(Eigen::Vector3d(0.0, -9.81, 0.0));

  pinocchio::JointIndex parent = 0;
  for(int i = 0; i < dof; ++i)
  {
    const Eigen::Vector3d translation = i == 0 ? Eigen::Vector3d::Zero()
                                               : Eigen::Vector3d(1.0, 0.0, 0.0);
    const pinocchio::SE3 placement(Eigen::Matrix3d::Identity(), translation);
    parent = model.addJoint(parent, pinocchio::JointModelRZ(), placement, "joint" + std::to_string(i));

    const double mass = 1.0 + 0.05 * static_cast<double>(i);
    const Eigen::Vector3d com(0.35 + 0.002 * static_cast<double>(i), 0.01, 0.0);
    model.appendBodyToJoint(parent, rigid_inertia(mass, com), pinocchio::SE3::Identity());
  }

  return model;
}

Eigen::VectorXd seeded_vector(int dof, double scale, int modulus)
{
  Eigen::VectorXd vector(dof);
  for(int i = 0; i < dof; ++i)
  {
    const double k = static_cast<double>(i + 1);
    vector[i] = scale * k - 0.01 * static_cast<double>(i % modulus);
  }
  return vector;
}

Eigen::VectorXd seeded_q(int dof)
{
  Eigen::VectorXd vector(dof);
  for(int i = 0; i < dof; ++i)
  {
    const double k = static_cast<double>(i + 1);
    vector[i] = 0.03 * k - 0.02 * static_cast<double>(i % 3);
  }
  return vector;
}

Eigen::VectorXd seeded_v(int dof)
{
  Eigen::VectorXd vector(dof);
  for(int i = 0; i < dof; ++i)
  {
    const double k = static_cast<double>(i + 1);
    vector[i] = 0.04 * k - 0.01 * static_cast<double>(i % 5);
  }
  return vector;
}

Eigen::VectorXd seeded_a(int dof)
{
  Eigen::VectorXd vector(dof);
  for(int i = 0; i < dof; ++i)
  {
    const double k = static_cast<double>(i + 1);
    vector[i] = 0.02 * k + 0.005 * static_cast<double>(i % 7);
  }
  return vector;
}

Eigen::VectorXd deterministic_tau(int dof)
{
  Eigen::VectorXd vector(dof);
  for(int i = 0; i < dof; ++i)
  {
    vector[i] = 0.1 + 0.03 * static_cast<double>(i + 1);
  }
  return vector;
}

Eigen::Matrix<double, 6, 1> motion_to_angular_first(const pinocchio::Motion & motion)
{
  Eigen::Matrix<double, 6, 1> out;
  out.template segment<3>(0) = motion.angular();
  out.template segment<3>(3) = motion.linear();
  return out;
}

Eigen::Matrix<double, 6, 1> force_to_angular_first(const pinocchio::Force & force)
{
  Eigen::Matrix<double, 6, 1> out;
  out.template segment<3>(0) = force.angular();
  out.template segment<3>(3) = force.linear();
  return out;
}

void emit_scalar(
  const std::string & group,
  const std::string & model_name,
  const std::string & quantity,
  int dof,
  int row,
  int col,
  double value)
{
  std::cout << "pinocchio," << group << "," << model_name << "," << quantity << "," << dof
            << "," << row << "," << col << "," << std::scientific << std::setprecision(17)
            << value << "\n";
}

void emit_vector(
  const std::string & group,
  const std::string & model_name,
  const std::string & quantity,
  int dof,
  const Eigen::VectorXd & values)
{
  for(Eigen::Index i = 0; i < values.size(); ++i)
  {
    emit_scalar(group, model_name, quantity, dof, static_cast<int>(i), 0, values[i]);
  }
}

template<int Size>
void emit_fixed_vector(
  const std::string & group,
  const std::string & model_name,
  const std::string & quantity,
  int dof,
  const Eigen::Matrix<double, Size, 1> & values)
{
  for(int i = 0; i < Size; ++i)
  {
    emit_scalar(group, model_name, quantity, dof, i, 0, values[i]);
  }
}

void perturb_state(Eigen::VectorXd & q)
{
  if(q.size() == 0)
    return;

  q[0] += 1.0e-12;
  if(q[0] > 1.0)
    q[0] = 0.03;
}

void run_spatial(const Config & config)
{
  const pinocchio::Motion motion_a(pinocchio::Motion::Vector6(4.0, 5.0, 6.0, 1.0, 2.0, 3.0));
  const pinocchio::Motion motion_b(pinocchio::Motion::Vector6(10.0, 11.0, 12.0, 7.0, 8.0, 9.0));
  const pinocchio::Force force(pinocchio::Force::Vector6(1.0, 1.1, 1.2, 0.7, 0.8, 0.9));
  const pinocchio::SE3 transform(rz(0.31), Eigen::Vector3d(0.4, -0.2, 0.7));
  const pinocchio::Inertia inertia = rigid_inertia(1.7, Eigen::Vector3d(0.25, -0.1, 0.05));

  emit("spatial", "spatial_ops", "cross_motion", 0, config, [&]() {
    return motion_a.cross(motion_b).toVector()[0];
  });
  emit("spatial", "spatial_ops", "cross_force", 0, config, [&]() {
    return motion_a.cross(force).toVector()[0];
  });
  emit("spatial", "spatial_ops", "apply_motion", 0, config, [&]() {
    return transform.act(motion_a).toVector()[0];
  });
  emit("spatial", "spatial_ops", "apply_force", 0, config, [&]() {
    return transform.act(force).toVector()[0];
  });
  emit("spatial", "spatial_ops", "inertia_apply_motion", 0, config, [&]() {
    return (inertia * motion_a).toVector()[0];
  });
}

void run_spatial_correctness()
{
  const pinocchio::Motion motion_a(pinocchio::Motion::Vector6(4.0, 5.0, 6.0, 1.0, 2.0, 3.0));
  const pinocchio::Motion motion_b(pinocchio::Motion::Vector6(10.0, 11.0, 12.0, 7.0, 8.0, 9.0));
  const pinocchio::Force force(pinocchio::Force::Vector6(1.0, 1.1, 1.2, 0.7, 0.8, 0.9));
  const pinocchio::SE3 transform(rz(0.31), Eigen::Vector3d(0.4, -0.2, 0.7));
  const pinocchio::Inertia inertia = rigid_inertia(1.7, Eigen::Vector3d(0.25, -0.1, 0.05));

  emit_fixed_vector(
    "spatial", "spatial_ops", "cross_motion", 0,
    motion_to_angular_first(motion_a.cross(motion_b)));
  emit_fixed_vector(
    "spatial", "spatial_ops", "cross_force", 0,
    force_to_angular_first(motion_a.cross(force)));
  emit_fixed_vector(
    "spatial", "spatial_ops", "apply_motion", 0,
    motion_to_angular_first(transform.act(motion_a)));
  emit_fixed_vector(
    "spatial", "spatial_ops", "apply_force", 0,
    force_to_angular_first(transform.act(force)));
  emit_fixed_vector(
    "spatial", "spatial_ops", "inertia_apply_motion", 0,
    force_to_angular_first(inertia * motion_a));
}

void run_dynamics_for(int dof, const std::string & model_name, const Config & config)
{
  const pinocchio::Model model = serial_revolute_model(dof);

  Eigen::VectorXd q = seeded_q(dof);
  Eigen::VectorXd v = seeded_v(dof);
  Eigen::VectorXd a = seeded_a(dof);
  Eigen::VectorXd tau = deterministic_tau(dof);

  {
    pinocchio::Data data(model);
    Eigen::VectorXd q_local = q;
    emit("dynamics", model_name, "rnea", dof, config, [&]() {
      perturb_state(q_local);
      return pinocchio::rnea(model, data, q_local, v, a)[0];
    });
  }

  {
    pinocchio::Data data(model);
    Eigen::VectorXd q_local = q;
    emit("dynamics", model_name, "crba", dof, config, [&]() {
      perturb_state(q_local);
      return pinocchio::crba(model, data, q_local)(0, 0);
    });
  }

  {
    pinocchio::Data data(model);
    Eigen::VectorXd q_local = q;
    emit("dynamics", model_name, "aba", dof, config, [&]() {
      perturb_state(q_local);
      return pinocchio::aba(model, data, q_local, v, tau)[0];
    });
  }
}

void run_dynamics_correctness_for(int dof, const std::string & model_name)
{
  const pinocchio::Model model = serial_revolute_model(dof);
  const Eigen::VectorXd q = seeded_q(dof);
  const Eigen::VectorXd v = seeded_v(dof);
  const Eigen::VectorXd a = seeded_a(dof);

  pinocchio::Data rnea_data(model);
  emit_vector("dynamics", model_name, "rnea_tau", dof, pinocchio::rnea(model, rnea_data, q, v, a));

  pinocchio::Data crba_data(model);
  const Eigen::MatrixXd & raw_m = pinocchio::crba(model, crba_data, q);
  Eigen::MatrixXd m = raw_m;
  m.template triangularView<Eigen::StrictlyLower>() =
    raw_m.transpose().template triangularView<Eigen::StrictlyLower>();
  for(int row = 0; row < dof; ++row)
  {
    for(int col = 0; col < dof; ++col)
    {
      emit_scalar("dynamics", model_name, "crba_m", dof, row, col, m(row, col));
    }
  }

  pinocchio::Data aba_data(model);
  emit_vector(
    "dynamics", model_name, "aba_qddot", dof,
    pinocchio::aba(model, aba_data, q, v, deterministic_tau(dof)));
}

void run_initialization_for(int dof, const std::string & model_name, const Config & config)
{
  emit("init", model_name, "build_model", dof, config, [&]() {
    const pinocchio::Model model = serial_revolute_model(dof);
    return model.gravity.linear()[1];
  });
}

void run_dynamics(const Config & config)
{
  run_dynamics_for(2, "chain_2dof", config);
  run_dynamics_for(6, "chain_6dof", config);
  run_dynamics_for(7, "chain_7dof", config);
  run_dynamics_for(12, "chain_12dof", config);
  run_dynamics_for(18, "chain_18dof", config);
  run_dynamics_for(30, "chain_30dof", config);
}

void run_correctness()
{
  run_spatial_correctness();
  run_dynamics_correctness_for(1, "chain_1dof");
  run_dynamics_correctness_for(2, "chain_2dof");
  run_dynamics_correctness_for(6, "chain_6dof");
  run_dynamics_correctness_for(7, "chain_7dof");
  run_dynamics_correctness_for(12, "chain_12dof");
  run_dynamics_correctness_for(18, "chain_18dof");
  run_dynamics_correctness_for(30, "chain_30dof");
}

void run_initialization(const Config & config)
{
  run_initialization_for(2, "chain_2dof", config);
  run_initialization_for(6, "chain_6dof", config);
  run_initialization_for(7, "chain_7dof", config);
  run_initialization_for(12, "chain_12dof", config);
  run_initialization_for(18, "chain_18dof", config);
  run_initialization_for(30, "chain_30dof", config);
}
}

int main(int argc, char ** argv)
{
  const Config config = parse_args(argc, argv);
  if(config.correctness)
  {
    std::cout << "implementation,group,model,quantity,dof,row,col,value\n";
    run_correctness();
    return 0;
  }

  std::cout << "implementation,group,model,algorithm,dof,iterations,total_ns,ns_per_iter,checksum\n";
  run_spatial(config);
  run_dynamics(config);
  if(config.include_init)
    run_initialization(config);
  return 0;
}
