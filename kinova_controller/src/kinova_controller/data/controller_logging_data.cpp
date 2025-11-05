#include "kinova_controller/data/controller_logging_data.hpp"


void ControllerLoggingData::resize(const int & nq, const int & nv)
{
    tau_c.resize(nv);
    est_tau_f.resize(nv);
    tau_f.resize(nv);
    nominal_theta.resize(nq);
    nominal_dtheta.resize(nv);
    nominal_ddtheta.resize(nv);
    e_dn.resize(nv);
    edot_dn.resize(nv);
    e_nr.resize(nv);
    edot_nr.resize(nv);
    u.resize(nv);
    q_des.resize(nq);
    theta_des.resize(nq);
}

void ControllerLoggingData::setZero()
{
    tau_c.setZero();
    est_tau_f.setZero();
    tau_f.setZero();
    nominal_theta.setZero();
    nominal_dtheta.setZero();
    nominal_ddtheta.setZero();
    e_dn.setZero();
    edot_dn.setZero();
    e_nr.setZero();
    edot_nr.setZero();
    u.setZero();
    q_des.setZero();
    theta_des.setZero();
}