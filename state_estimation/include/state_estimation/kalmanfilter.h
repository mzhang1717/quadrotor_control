#ifndef TAG_FILTERKALMANFILTER_H
#define TAG_FILTERKALMANFILTER_H

#include <TooN/TooN.h>
#include <TooN/helpers.h>
#include <TooN/Cholesky.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace tag {

/**
@defgroup kalmanfiltergroup Kalman Filter
A basic Kalman filter implementation and various state, process models and measurement functions.

Template class providing a basic implementation of the Kalman filter.
The state and the process model are both template parameter classes
to keep it flexible. Both parameters have to implement a certain interface to make the filter work.
@code
class State {
public:
    const static int STATE_DIMENSION =  ??;  // dimension of filter state
    TooN::Matrix<STATE_DIMENSION> covariance;      // covariance of filter state
};

class Model {
public:
    // return process model jacobian for a given state and time delta (typically A)
    const TooN::Matrix<State::STATE_DIMENSION> & getJacobian(const State & state, double dt);
    // update the state for a given time delta (not all states are actually x = Ax, therefore this is a function)
    void updateState( State & state, const double dt );
    // return process noise matrix for given time delta (typically Q)
    const TooN::Matrix<State::STATE_DIMENSION> & getNoiseCovariance( double dt );
    // update the state from an innovation. the innovation was computed by the filter based on measurement etc.
    void updateFromMeasurement( State & state, const TooN::Vector<State::STATE_DIMENSION> & innovation );
};
@endcode
Measurements are incorporated through the template member function
template<class Measurement> void KalmanFilter<class State, class Model>::filter(Measurement & m);
where class Measurement also has to implement a certain protocol:
@code
class Measurement {
public:
    static const int M_DIMENSION =  ??;  // dimension of measurement
    // return measurement jacobian, from state -> measurement
    const Matrix<M_DIMENSION,State::STATE_DIMENSION> & getMeasurementJacobian( const State & state );
    // return measurement noise covariance
    const Matrix<M_DIMENSION> & getMeasurementCovariance( const State & state );
    // return the innovation, the difference between actual measurement and the measurement prediction based on the state
    const Vector<M_DIMENSION> & getInnovation( const State & state );
};
@endcode
All of the member functions take the state as parameters, because the returned values are typically
functions of the state in some form.

Basically, the three classes State, Model, Measurement have to know about each other and work together.
However, splitting them apart allows one to change models and use multiple measurement functions for a
single kind of state. That simplifies sensor fusion and SCAAT style use of the Kalman filter.

The following example demonstrates how to use the filter classes.
@code
tag::KalmanFilter<tag::ConstantVelocity::State, tag::ConstantVelocity::Model> filter;
filter.state.pose = // Initial pose
filter.state.covariance = // Initial covariance

while(true){
    double deltaT = 0.1; // interval between measurements
    filter.predict( deltaT );

    tag::IncrementalPose m;
    m.measurement = // update vector = ln() of the correction SE3
    m.covariance =  // your measurement covariance
    filter.filter(m);
}
@endcode

Note, that all the return values from the various classes are const references. This avoids any unnecessary copying of data.
You can also return types that may be stored in const references, such as non-const references and return values.
*/

/**
the basic template class implementing the Kalman Filter, see @ref kalmanfiltergroup documentation for details.
@ingroup kalmanfiltergroup
*/
template<class State, class Model>
class KalmanFilter{
public:

    typedef State state_type;
    typedef Model model_type;
    
    ros::NodeHandle nh;
    ros::Publisher nisPub;
    
    enum CHI_VAL{CHI_99, CHI_95, CHI_90} chiPct;
    double chiTable[3][6];

    KalmanFilter(){
        identity = TooN::Identity;
        nisPub = nh.advertise<std_msgs::Float64>("nis",1);
        
        chiPct = CHI_90;
        
        chiTable[CHI_99][0] = 1e-10; // don't use this!!!
        chiTable[CHI_99][1] = 0.02;
        chiTable[CHI_99][2] = 0.115;
        chiTable[CHI_99][3] = 0.297;
        chiTable[CHI_99][4] = 0.554;
        chiTable[CHI_99][5] = 0.872;
        
        chiTable[CHI_95][0] = 0.004;
        chiTable[CHI_95][1] = 0.103;
        chiTable[CHI_95][2] = 0.352;
        chiTable[CHI_95][3] = 0.711;
        chiTable[CHI_95][4] = 1.145;
        chiTable[CHI_95][5] = 1.635;
        
        chiTable[CHI_90][0] = 0.016;
        chiTable[CHI_90][1] = 0.211;
        chiTable[CHI_90][2] = 0.584;
        chiTable[CHI_90][3] = 1.064;
        chiTable[CHI_90][4] = 1.610;
        chiTable[CHI_90][5] = 2.204;
    }

    /// predicts the state by applying the process model over the time interval dt
    /// @param[in] dt time interval
    
    void predict(double dt){
        //state.covariance = TooN::transformCovariance(model.getJacobian( state, dt ), state.covariance) + model.getNoiseCovariance( dt );
        const TooN::Matrix<State::STATE_DIMENSION> & A = model.getJacobian( state, dt );
        state.covariance = A * state.covariance * A.T() + model.getNoiseCovariance( dt );
        TooN::Symmetrize(state.covariance);
        model.updateState( state, dt );
    }

    template<class Input> void predict(Input& input, double dt){
        //state.covariance = TooN::transformCovariance(model.getJacobian( state, dt ), state.covariance) + model.getNoiseCovariance( dt );
        const TooN::Matrix<State::STATE_DIMENSION> & A = model.getJacobian( state, input, dt );
        state.covariance = A * state.covariance * A.T() + model.getNoiseCovariance( input, dt );
        TooN::Symmetrize(state.covariance);
        model.updateState( state, input, dt );
    }

    /// incorporates a measurement
    /// @param[in] m the measurement to add to the filter state
    template<class Measurement> void filter(Measurement & m){
        const TooN::Matrix<Measurement::M_DIMENSION,State::STATE_DIMENSION> & H = m.getMeasurementJacobian( state );
        const TooN::Matrix<Measurement::M_DIMENSION> & R = m.getMeasurementCovariance( state );
        const TooN::Vector<Measurement::M_DIMENSION> & innovation = m.getInnovation( state );
        const TooN::Matrix<State::STATE_DIMENSION, Measurement::M_DIMENSION> P12 = state.covariance * H.T();
        const TooN::Matrix<Measurement::M_DIMENSION> S = H * P12 + R;
         
        TooN::Cholesky<Measurement::M_DIMENSION> denom(S);
        
        if(Measurement::M_DIMENSION == 1)
        {
          // Normalized innovation squared
          double nis = innovation * denom.backsub(innovation);
          std::cout<<"NIS: "<<nis;
          
          std_msgs::Float64 nisMsg;
          nisMsg.data = nis;
          nisPub.publish(nisMsg);
          
          double chiLimit = chiTable[chiPct][Measurement::M_DIMENSION-1];
          if(nis > chiLimit)
          {
            std::cout<<"  <<--bad!"<<std::endl;
            //return;
          }
          
          std::cout<<std::endl;
        }
        
        /*
        // Debug code
        const TooN::Matrix<State::STATE_DIMENSION, Measurement::M_DIMENSION> KalmanGain = P12 * denom.get_inverse();
        std::cout<<"Kalman Gain: "<<std::endl<<KalmanGain<<std::endl;
        
        state.covariance = state.covariance - KalmanGain*H*state.covariance;
        TooN::Symmetrize(state.covariance);
        const TooN::Vector<State::STATE_DIMENSION> stateInnovation = KalmanGain*innovation;
        // End debug
        */
        
        // Original code
        state.covariance = state.covariance - P12 * denom.backsub(P12.T());
        // TooN::Symmetrize(state.covariance);  // not necessary, above seems to be good enough
        const TooN::Vector<State::STATE_DIMENSION> stateInnovation = P12 * denom.backsub(innovation);
        // End original
        
        
        model.updateFromMeasurement( state, stateInnovation );
    }

    /// identity matrix of the right size, used in the measurement equations
    TooN::Matrix<State::STATE_DIMENSION> identity;
    /// the current state of the filter
    State state;
    /// the process model used by the filter
    Model model;
};

} // namespace tag

#endif
