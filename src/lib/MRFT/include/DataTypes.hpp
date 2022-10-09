#ifndef DATATYPES_HPP
#define DATATYPES_HPP

namespace HEAR{

    enum TYPE{
        NA,
        Float,
        Float3,
        FloatVec,
        RotMat,
        QUAT
    };
    enum BLOCK_ID{
        EXT_IP,
        EXT_OP,
        PID,
        MRFT,
        SUM,
        SUM3,
        GAIN,
        MUX3,
        MULTIPLY,
        DEMUX3,
        CONSTANT,
        DIFFERENTIATOR,
        HOLDVAL,
        BW_FILT2,
        SATURATION,
        EUL2ROT,
        FOH,
        ROT2EUL,
        ROT2QUAT,
        QUAT2ROT,
        TOHORIZON,
        FROMHORIZON,
        FORCE2ROT,
        ROTDIFF2ROD,
        HEXAACTUATIONSYSTEM,
        QUADACTUATIONSYSTEM,
        SWITCH,
        INVERTED_SWITCH,
        INVERTED_SWITCH3,
        MEDIAN_FILTER,
        BATTERYMONITOR,
        RC_IN,
        NAVIO2IMU,
        RADIO_CONTROLLER,
        TEMPLATE
    };
    enum IOTYPE{
        INPUT =0,
        OUTPUT =1
    };
    enum TRIG_TYPE{
        RESET,
        UPDATE
    };
    enum UPDATE_MSG_TYPE{
        PID_UPDATE,
        MRFT_UPDATE,
        BB_UPDATE,
        SWITCH_TRIG,
        BOOL_MSG,
        TEMPLATE_MSG
    };
    enum SWITCH_STATE{
        OFF,
        ON,
        TOGGLE
    };
    enum SWITCH_ID{
        MRFT_SW
    };
    enum PID_ID{
        PID_X,
        PID_Y,
        PID_Z,
        PID_ROLL,
        PID_PITCH,
        PID_YAW,
        PID_YAW_RATE
    };

    enum MRFT_ID{
        MRFT_X=14, MRFT_Y=15, MRFT_Z=16, MRFT_ROLL=17, MRFT_PITCH=18, 
					MRFT_YAW=19, MRFT_YAW_RATE=20
    };

    class PID_parameters {
    public:
        int id;
        float kp=1, ki=0, kd=0, kdd=0, anti_windup=0;
        bool en_pv_derivation = false;
    };
    
    class MRFT_parameters {
    public:
        int id;	   
        float beta=0, relay_amp=0;
        int no_switch_delay_in_ms=20, num_of_peak_conf_samples=5;
    };

    class UpdateMsg{
        public:
            virtual ~UpdateMsg(){}
            virtual UPDATE_MSG_TYPE getType() const =0;
            virtual UpdateMsg* copy() const =0;
    };

    class PID_UpdateMsg: UpdateMsg{
        public:
            PID_parameters param;
            UPDATE_MSG_TYPE getType()const {return UPDATE_MSG_TYPE::PID_UPDATE;}
            UpdateMsg* copy() const{
               auto Copy = new PID_UpdateMsg;
               *Copy = *this;
               return Copy;
            } 
    };
    class MRFT_UpdateMsg : UpdateMsg{
        public:
            MRFT_parameters param;
            UPDATE_MSG_TYPE getType()const {return UPDATE_MSG_TYPE::MRFT_UPDATE;}
            UpdateMsg* copy() const{
               auto Copy = new MRFT_UpdateMsg;
               *Copy = *this;
               return Copy;
            }
    };

    class SwitchMsg : UpdateMsg{
        public:
            int sw_id;
            SWITCH_STATE sw_state = SWITCH_STATE::OFF;
            UPDATE_MSG_TYPE getType() const {return UPDATE_MSG_TYPE::SWITCH_TRIG;}
            UpdateMsg* copy() const{
                auto Copy = new SwitchMsg;
                *Copy = *this;
                return Copy;
            }
    };

    class BoolMsg : UpdateMsg{
        public:
            bool data = false;
            UPDATE_MSG_TYPE getType()const {return UPDATE_MSG_TYPE::BOOL_MSG;}
            UpdateMsg* copy() const{
                auto Copy = new BoolMsg;
                *Copy = *this;
                return Copy;
            }
    };

    class TemplateMsg : UpdateMsg{
        public:
            float param = 0.1;
            UPDATE_MSG_TYPE getType() const {
                return UPDATE_MSG_TYPE::TEMPLATE_MSG;
            }
            UpdateMsg* copy() const{
                auto Copy = new TemplateMsg;
                *Copy = *this;
                return Copy;
            }
    };

    class float3{
    public:
        float x=0, y=0, z=0;
    };

}

#endif