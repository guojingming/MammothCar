#ifndef GNSSFILTER_H
#define GNSSFILTER_H

#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/random.h>

#define DELTA_TIME 0.05f
#define MODEL_NOISE_STD 1.0f
#define MEASURE_NOISE_STD 3.0f

class GNSSFilter: public mrpt::bayes::CKalmanFilterCapable<4, 4, 0, 0>
{
  public:
    GNSSFilter(float init_x, float init_y, float init_vx, float init_vy, float measure_noise, float model_noise);
    virtual ~GNSSFilter();

    void Process(double DeltaTime, double obsx, double obsy, double obsvx, double obsvy);
    void getState(KFVector &xkk, KFMatrix &pkk)
    {
      xkk = m_xkk;
      pkk = m_pkk;
    }
  protected:
    float m_obsx, m_obsy, m_obsvx, m_obsvy;
    float m_deltaTime;
    float m_model_noise;
    float m_measure_noise;
    
    void OnGetAction(KFArray_ACT & out_u) const;
    void OnTransitionModel(const KFArray_ACT &in_u, KFArray_VEH &inout_x, bool &out_skipPrediction) const;
    void OnTransitionJacobian(KFMatrix_VxV  &out_F) const;
    void OnTransitionNoise(KFMatrix_VxV &out_Q) const;
    void OnGetObservationNoise(KFMatrix_OxO &out_R) const;

    void OnGetObservationsAndDataAssociation(
	vector_KFArray_OBS    &Z,
	mrpt::vector_int     &data_association,
	const vector_KFArray_OBS   &all_predictions,
	const KFMatrix              &S,
	const mrpt::vector_size_t  &lm_indices_in_S,
	const KFMatrix_OxO          &R
	);
    void OnObservationModel(const mrpt::vector_size_t &idx_landmarks_to_predict, vector_KFArray_OBS &out_predictions) const;
    virtual void OnObservationJacobians(
	const size_t &idx_landmark_to_predict,
	KFMatrix_OxV &Hx,
	KFMatrix_OxF &Hy
	) const;
};




#endif
