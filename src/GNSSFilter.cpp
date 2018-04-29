#include "GNSSFilter.h"


GNSSFilter::GNSSFilter(float init_x, float init_y, float init_vx, float init_vy, float measure_noise, float model_noise)
{
  m_model_noise = model_noise;
  m_measure_noise = measure_noise;

  KF_options.method = mrpt::bayes::kfEKFNaive; 
  m_xkk.resize(4);
  m_xkk[0] = init_x;
  m_xkk[1] = init_y;
  m_xkk[2] = init_vx;
  m_xkk[3] = init_vy;

  m_pkk.setSize(4,4);
  m_pkk.unit();
  m_pkk = 50 * m_pkk;

}
GNSSFilter::~GNSSFilter()
{

}
void GNSSFilter::Process(double DeltaTime, double obsx, double obsy, double obsvx, double obsvy)
{
  m_deltaTime = DeltaTime;
  m_obsx = obsx;
  m_obsy = obsy;
  m_obsvx = obsvx;
  m_obsvy = obsvy;

  runOneKalmanIteration();
}
void GNSSFilter::OnGetAction(KFArray_ACT & out_u) const
{
  
}
void GNSSFilter::OnTransitionModel(const KFArray_ACT &in_u, KFArray_VEH &inout_x, bool &out_skipPrediction) const
{
  //x,y,vx,vy 
  inout_x[0] += inout_x[3] * m_deltaTime;
  inout_x[1] += inout_x[4] * m_deltaTime;
  inout_x[2] = inout_x[2];
  inout_x[3] = inout_x[3];
}
void GNSSFilter::OnTransitionJacobian(KFMatrix_VxV  &F) const
{
  F.unit();
  F(0,2) = m_deltaTime;
  F(1,3) = m_deltaTime;
}
void GNSSFilter::OnTransitionNoise(KFMatrix_VxV &Q) const
{
  Q.unit();
  Q *= m_model_noise;
}

void GNSSFilter::OnGetObservationNoise(KFMatrix_OxO &R) const
{
  R.unit();
  R *= m_measure_noise;
}

void GNSSFilter::OnGetObservationsAndDataAssociation(
    vector_KFArray_OBS    &out_z,
    mrpt::vector_int                  &data_association,
    const vector_KFArray_OBS   &all_predictions,
    const KFMatrix              &S,
    const mrpt::vector_size_t         &lm_indices_in_S,
    const KFMatrix_OxO          &R
    )
{
  out_z.resize(1);
  out_z[0][0] = m_obsx;
  out_z[0][1] = m_obsy;
  out_z[0][2] = m_obsvx;
  out_z[0][3] = m_obsvy;
  
}
void GNSSFilter::OnObservationModel(const mrpt::vector_size_t &idx_landmarks_to_predict, vector_KFArray_OBS &out_predictions) const
{
  out_predictions.resize(1);
  out_predictions[0][0] = m_xkk[0];
  out_predictions[0][1] = m_xkk[1];
  out_predictions[0][2] = m_xkk[2];
  out_predictions[0][3] = m_xkk[3];
}
void GNSSFilter::OnObservationJacobians(
    const size_t &idx_landmark_to_predict,
    KFMatrix_OxV &Hx,
    KFMatrix_OxF &Hy
    ) const
{
  Hx.zeros();
  Hx(0,0) = 1;
  Hx(1,1) = 1;
  Hx(2,2) = 1;
  Hx(3,3) = 1;

}
