// ***************************************************************************************************************************************************
#include "Xicro_dffdrive_ID_1.h"

#define __SEND_UART(x,y) _serial->write(x,y);


Xicro::Xicro(){
}

void Xicro::begin(__UART_TYPE *SerialObject){


    _serial=SerialObject;
    // gen_void_begin

  

// get
}     
uint8_t Xicro::_getcrc(uint8_t crc,uint8_t *data,uint16_t len){
    if(len>1){
        for (int i = 0; i < len; i++){
            crc = _CRC_8_TABLE[crc ^ data[i]];
        }
    }else if(len!=0){
        crc = _CRC_8_TABLE[crc ^ (uint8_t)*data];
    }
    
    return crc;
}

void Xicro::_Sendstart(){
    __SEND_UART(_start,4);
}
void Xicro::_Sendstop(){
    _crc=_getcrc(_crc,_stop,2);
    __SEND_UART(_stop,2);
}
void Xicro::_Sendcontinue(){
    _crc=_getcrc(_crc,_continue,2);
    __SEND_UART(_continue,2);
}
void Xicro::_SendSignature(uint8_t Idmcu,uint8_t Mode){
   uint8_t q[1]={0};
   q[0]=Idmcu<<4;
   q[0]=q[0]|Mode;
   _crc=_getcrc(_crc, &q[0],1);
   __SEND_UART(q,1);
}
void Xicro::_SendUint8(uint8_t *data,uint8_t len){
    if(len>1){
        uint8_t buff[2]={9,0};
        buff[1]=len & 0xFF;
        __SEND_UART(buff,2);
        _crc=_getcrc(_crc,buff,2);
        _crc=_getcrc(_crc,data,len);
        __SEND_UART(data,len);
    }else{
        uint8_t buff[2]={8,0};
        buff[1]=(uint8_t)*data & 0xFF;
        _crc=_getcrc(_crc,buff,2);
        __SEND_UART(buff,2);
    }
    

}
void Xicro::_SendUint16(uint16_t *data,uint8_t len){
    if(len>1){
        const uint16_t sizee=len*2;
        uint8_t buff[2]={17,0};
        buff[1]=len & 0xFF;
        _crc=_getcrc(_crc,buff,2);
        __SEND_UART(buff,2);
        uint8_t sp[sizee];
        for (int i=0;i<len;i++){
            sp[(i*2)+1] = (uint16_t)data[i] >> 0 & 0xFF;
            sp[(i*2)]   = (uint16_t)data[i] >> 8 & 0xFF;
            
        }
        _crc=_getcrc(_crc,sp,sizee);
        __SEND_UART(sp,sizee);

    }else{
        uint8_t buff[3]={16,0,0};
        buff[2] = (uint16_t)*data >> 0 & 0xFF;
        buff[1] = (uint16_t)*data >> 8 & 0xFF;
        _crc=_getcrc(_crc,buff,3);
        __SEND_UART(buff,3);
    }
    
}
void Xicro::_SendUint32(uint32_t *data,uint8_t len){
   
    if(len>1){
        const uint16_t sizee=len*4;
        uint8_t buff[2]={33,0};
        buff[1]=len & 0xFF;
        _crc=_getcrc(_crc,buff,2);
        __SEND_UART(buff,2);
        uint8_t sp[sizee];
        for (int i=0;i<len;i++){
            sp[(i*4)+3] = (uint32_t)data[i] >> 0 & 0xFF;
            sp[(i*4)+2] = (uint32_t)data[i] >> 8 & 0xFF;
            sp[(i*4)+1] = (uint32_t)data[i] >> 16 & 0xFF;
            sp[(i*4)]   = (uint32_t)data[i] >> 24 & 0xFF;     
        }
        _crc=_getcrc(_crc,sp,sizee);
        __SEND_UART(sp,sizee);

    }else{
        uint8_t buff[5]={32,0,0,0,0};
        buff[4] = (uint32_t)*data >> 0 & 0xFF;
        buff[3] = (uint32_t)*data >> 8 & 0xFF;
        buff[2] = (uint32_t)*data >> 16 & 0xFF;
        buff[1] = (uint32_t)*data >> 24 & 0xFF;
        _crc=_getcrc(_crc,buff,5);
        __SEND_UART(buff,5);
        
    }
}
void Xicro::_SendUint64(uint64_t *data,uint8_t len){
    if(len>1){
        const uint16_t sizee=len*8;
        uint8_t buff[2]={65,0};
        buff[1]=len & 0xFF;
        _crc=_getcrc(_crc,buff,2);
        __SEND_UART(buff,2);
        uint8_t sp[sizee];
        for (int i=0;i<len;i++){
            sp[(i*8)+7] = (uint64_t)data[i] >> 0  & 0xFF;
            sp[(i*8)+6] = (uint64_t)data[i] >> 8  & 0xFF;
            sp[(i*8)+5] = (uint64_t)data[i] >> 16 & 0xFF;
            sp[(i*8)+4] = (uint64_t)data[i] >> 24 & 0xFF;  
            sp[(i*8)+3] = (uint64_t)data[i] >> 32 & 0xFF;
            sp[(i*8)+2] = (uint64_t)data[i] >> 40 & 0xFF;
            sp[(i*8)+1] = (uint64_t)data[i] >> 48 & 0xFF;
            sp[(i*8)]   = (uint64_t)data[i] >> 56 & 0xFF;     
        }
        _crc=_getcrc(_crc,sp,sizee);
        __SEND_UART(sp,sizee);

    }else{
        uint8_t buff[9]={64,0,0,0,0,0,0,0,0};
        buff[8] = (uint64_t)*data >> 0  & 0xFF;
        buff[7] = (uint64_t)*data >> 8  & 0xFF;
        buff[6] = (uint64_t)*data >> 16 & 0xFF;
        buff[5] = (uint64_t)*data >> 24 & 0xFF;
        buff[4] = (uint64_t)*data >> 32 & 0xFF;
        buff[3] = (uint64_t)*data >> 40 & 0xFF;
        buff[2] = (uint64_t)*data >> 48 & 0xFF;
        buff[1] = (uint64_t)*data >> 56 & 0xFF;
        _crc=_getcrc(_crc,buff,9);
        __SEND_UART(buff,9);
        
    }
}
void Xicro::_SendInt8(int8_t *data,uint8_t len){
    if(len>1){
        uint8_t buff[2]={19,0};
        buff[1]=len & 0xFF;
        __SEND_UART(buff,2);
        _crc=_getcrc(_crc,buff,2);       
        _crc=_getcrc(_crc,(uint8_t*)data,len);
        __SEND_UART((uint8_t*)data,len);
    }else{
        uint8_t buff[2]={18,0};
        buff[1]=(uint8_t)*data & 0xFF;
        _crc=_getcrc(_crc,buff,2);
        __SEND_UART(buff,2);
    }
}
void Xicro::_SendInt16(int16_t *data,uint8_t len){
    if(len>1){
        const uint16_t sizee=len*2;
        uint8_t buff[2]={117,0};
        buff[1]=len & 0xFF;
        _crc=_getcrc(_crc,buff,2);
        __SEND_UART(buff,2);
        uint8_t sp[sizee];
        for (int i=0;i<len;i++){
            sp[(i*2)+1] = (uint16_t)data[i] >> 0 & 0xFF;
            sp[(i*2)]   = (uint16_t)data[i] >> 8 & 0xFF;
            
        }
        _crc=_getcrc(_crc,sp,sizee);
        __SEND_UART(sp,sizee);

    }else{
        uint8_t buff[3]={116,0,0};
        buff[2] = (uint16_t)*data >> 0 & 0xFF;
        buff[1] = (uint16_t)*data >> 8 & 0xFF;
        _crc=_getcrc(_crc,buff,3);
        __SEND_UART(buff,3);
    }
}
void Xicro::_SendInt32(int32_t *data,uint8_t len){
    if(len>1){
        const uint16_t sizee=len*4;
        uint8_t buff[2]={133,0};
        buff[1]=len & 0xFF;
        _crc=_getcrc(_crc,buff,2);
        __SEND_UART(buff,2);
        uint8_t sp[sizee];
        for (int i=0;i<len;i++){
            sp[(i*4)+3] = (uint32_t)data[i] >> 0 & 0xFF;
            sp[(i*4)+2] = (uint32_t)data[i] >> 8 & 0xFF;
            sp[(i*4)+1] = (uint32_t)data[i] >> 16 & 0xFF;
            sp[(i*4)]   = (uint32_t)data[i] >> 24 & 0xFF;     
        }
        _crc=_getcrc(_crc,sp,sizee);
        __SEND_UART(sp,sizee);

    }else{
        uint8_t buff[5]={132,0,0,0,0};
        buff[4] = (uint32_t)*data >> 0 & 0xFF;
        buff[3] = (uint32_t)*data >> 8 & 0xFF;
        buff[2] = (uint32_t)*data >> 16 & 0xFF;
        buff[1] = (uint32_t)*data >> 24 & 0xFF;
        _crc=_getcrc(_crc,buff,5);
        __SEND_UART(buff,5);
    }
}
void Xicro::_SendInt64(int64_t *data,uint8_t len){
    if(len>1){
        const uint16_t sizee=len*8;
        uint8_t buff[2]={165,0};
        buff[1]=len & 0xFF;
        _crc=_getcrc(_crc,buff,2);
        __SEND_UART(buff,2);
        uint8_t sp[sizee];
        for (int i=0;i<len;i++){
            sp[(i*8)+7] = (uint64_t)data[i] >> 0  & 0xFF;
            sp[(i*8)+6] = (uint64_t)data[i] >> 8  & 0xFF;
            sp[(i*8)+5] = (uint64_t)data[i] >> 16 & 0xFF;
            sp[(i*8)+4] = (uint64_t)data[i] >> 24 & 0xFF;  
            sp[(i*8)+3] = (uint64_t)data[i] >> 32 & 0xFF;
            sp[(i*8)+2] = (uint64_t)data[i] >> 40 & 0xFF;
            sp[(i*8)+1] = (uint64_t)data[i] >> 48 & 0xFF;
            sp[(i*8)]   = (uint64_t)data[i] >> 56 & 0xFF;     
        }
        _crc=_getcrc(_crc,sp,sizee);
        __SEND_UART(sp,sizee);

    }else{
        uint8_t buff[9]={164,0,0,0,0,0,0,0,0};
        buff[8] = (uint64_t)*data >> 0  & 0xFF;
        buff[7] = (uint64_t)*data >> 8  & 0xFF;
        buff[6] = (uint64_t)*data >> 16 & 0xFF;
        buff[5] = (uint64_t)*data >> 24 & 0xFF;
        buff[4] = (uint64_t)*data >> 32 & 0xFF;
        buff[3] = (uint64_t)*data >> 40 & 0xFF;
        buff[2] = (uint64_t)*data >> 48 & 0xFF;
        buff[1] = (uint64_t)*data >> 56 & 0xFF;
        _crc=_getcrc(_crc,buff,9);
        __SEND_UART(buff,9);
        
    }
}
void Xicro::_SendFloat32(float *data,uint8_t len){
    if(len>1){
        uint8_t buff[2]={112,0};
        buff[1]=len & 0xFF;
        _crc=_getcrc(_crc,buff,2);
        __SEND_UART(buff,2);
        for(int i=0;i<len;i++){
            byte * bb = (byte *) &data[i];
            _crc=_getcrc(_crc,bb,4);
            __SEND_UART(bb,4);
        }

    }else{
        byte * bb = (byte *) &*data;
        uint8_t buff[1]={111};
        _crc=_getcrc(_crc,&buff[0],1);
        _crc=_getcrc(_crc,bb,4);
        __SEND_UART(buff,1);
        __SEND_UART(bb,4);
    }
    
}
void Xicro::_SendString(__STR_TYPE *data,uint8_t len){
    uint8_t buffE[2]={42,126};
    if(len>1){
        uint8_t buffS[2]={243,0};
        buffS[1]=len & 0xFF;
        _crc=_getcrc(_crc,buffS,2);
        __SEND_UART(buffS,2);
        for(int j=0;j<len;j++){
            _crc=_getcrc(_crc,(uint8_t*)&data[j][0],data[j].length());
            _crc=_getcrc(_crc,buffE,2);
            __SEND_UART((uint8_t*)&data[j][0],data[j].length());
            __SEND_UART(buffE,2);
        }

    }else{
        uint8_t buffS[1]={242};
        _crc=_getcrc(_crc,&buffS[0],1);
        _crc=_getcrc(_crc,(uint8_t*)&data[0][0],data[0].length());
        _crc=_getcrc(_crc,buffE,2);
        __SEND_UART(buffS,1);
        __SEND_UART((uint8_t*)&data[0][0],data[0].length());
        __SEND_UART(buffE,2);
    }
    

}
void Xicro::_SendBool(bool *data,uint8_t len,bool flagNext){ //flagnext=1=continue,flagnext=0=stop
    if(len>1){
        uint8_t sizee=2;
        sizee=sizee+ ceil((float)len/8.0);
        uint8_t buff[sizee];
        buff[0]=89;
        buff[1]=len & 0xFF;
        for(int i=2;i<sizee;i++){
            buff[i]=0;
        }
        uint8_t onbyte=2;
        uint8_t j=0;
        for(int i;i<len;i++){
            data[i]=data[i] & 0x01;
            buff[onbyte]= buff[onbyte] | (data[i] << j);
             
            if(j == 7){
                onbyte=onbyte+1;
                j=0;
            }else{
                j=j+1;
            }
            
        }
        _crc=_getcrc(_crc,buff,sizee);
        __SEND_UART(buff,sizee);

    }else{
        uint8_t buff[3]={88,0,0};
        if(flagNext){
            if((bool)data){
                buff[1]=250;
                buff[2]=250;
            }else{
                buff[1]=47;
                buff[2]=47;
            }
        }else{
            if((bool)data){
                buff[1]=254;
                buff[2]=254;
            }else{
                buff[1]=127;
                buff[2]=127;
            }
        }
        _crc=_getcrc(_crc,buff,3);
        __SEND_UART(buff,3);
    }
}
void Xicro::_Sendcrc(){
    __SEND_UART(&_crc,1);
}
void Xicro::_SendIdTopic(uint8_t IdTopic){
    _crc=_getcrc(_crc,&IdTopic,1);
    __SEND_UART(&IdTopic,1);
}
void Xicro::_SendEmpty(){
    uint8_t em=254;
    _crc=_getcrc(_crc,&em,1);
    __SEND_UART(&em,1);
}
void Xicro::Test(){
    

}









    _crc=0;
    _SendSignature(1,4);
    _SendIdTopic(1);
    _SendInt32((int32_t*)&Publisher_diff_encoder_tick.message.header.stamp.sec,1);
    _Sendcontinue();
    _SendUint32((uint32_t*)&Publisher_diff_encoder_tick.message.header.stamp.nanosec,1);
    _Sendcontinue();
    _SendString(&Publisher_diff_encoder_tick.message.header.frame_id,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_encoder_tick.message.vector.x,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_encoder_tick.message.vector.y,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_encoder_tick.message.vector.z,1);
    _Sendstop();
    _Sendcrc();
void Xicro::publish_diff_speed_req(){
    _crc=0;
    _SendSignature(1,4);
    _SendIdTopic(2);
    _SendInt32((int32_t*)&Publisher_diff_speed_req.message.header.stamp.sec,1);
    _Sendcontinue();
    _SendUint32((uint32_t*)&Publisher_diff_speed_req.message.header.stamp.nanosec,1);
    _Sendcontinue();
    _SendString(&Publisher_diff_speed_req.message.header.frame_id,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_speed_req.message.vector.x,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_speed_req.message.vector.y,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_speed_req.message.vector.z,1);
    _Sendstop();
    _Sendcrc();
void Xicro::publish_diff_PWM_cmd(){
    _crc=0;
    _SendSignature(1,4);
    _SendIdTopic(3);
    _SendInt32((int32_t*)&Publisher_diff_PWM_cmd.message.header.stamp.sec,1);
    _Sendcontinue();
    _SendUint32((uint32_t*)&Publisher_diff_PWM_cmd.message.header.stamp.nanosec,1);
    _Sendcontinue();
    _SendString(&Publisher_diff_PWM_cmd.message.header.frame_id,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_PWM_cmd.message.vector.x,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_PWM_cmd.message.vector.y,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_PWM_cmd.message.vector.z,1);
    _Sendstop();
    _Sendcrc();
void Xicro::publish_diff_speed_cmd(){
    _crc=0;
    _SendSignature(1,4);
    _SendIdTopic(4);
    _SendInt32((int32_t*)&Publisher_diff_speed_cmd.message.header.stamp.sec,1);
    _Sendcontinue();
    _SendUint32((uint32_t*)&Publisher_diff_speed_cmd.message.header.stamp.nanosec,1);
    _Sendcontinue();
    _SendString(&Publisher_diff_speed_cmd.message.header.frame_id,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_speed_cmd.message.vector.x,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_speed_cmd.message.vector.y,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_speed_cmd.message.vector.z,1);
    _Sendstop();
    _Sendcrc();
void Xicro::publish_diff_imu(){
    _crc=0;
    _SendSignature(1,4);
    _SendIdTopic(5);
    _SendInt32((int32_t*)&Publisher_diff_imu.message.header.stamp.sec,1);
    _Sendcontinue();
    _SendUint32((uint32_t*)&Publisher_diff_imu.message.header.stamp.nanosec,1);
    _Sendcontinue();
    _SendString(&Publisher_diff_imu.message.header.frame_id,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu.message.vector.x,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu.message.vector.y,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu.message.vector.z,1);
    _Sendstop();
    _Sendcrc();
void Xicro::publish_diff_imu_raw(){
    _crc=0;
    _SendSignature(1,4);
    _SendIdTopic(6);
    _SendInt32((int32_t*)&Publisher_diff_imu_raw.message.header.stamp.sec,1);
    _Sendcontinue();
    _SendUint32((uint32_t*)&Publisher_diff_imu_raw.message.header.stamp.nanosec,1);
    _Sendcontinue();
    _SendString(&Publisher_diff_imu_raw.message.header.frame_id,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.orientation.x,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.orientation.y,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.orientation.z,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.orientation.w,1);
    _Sendcontinue();
    _SendFloat32(Publisher_diff_imu_raw.message.orientation_covariance,9);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.angular_velocity.x,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.angular_velocity.y,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.angular_velocity.z,1);
    _Sendcontinue();
    _SendFloat32(Publisher_diff_imu_raw.message.angular_velocity_covariance,9);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.linear_acceleration.x,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.linear_acceleration.y,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_imu_raw.message.linear_acceleration.z,1);
    _Sendcontinue();
    _SendFloat32(Publisher_diff_imu_raw.message.linear_acceleration_covariance,9);
    _Sendstop();
    _Sendcrc();
void Xicro::publish_diff_encoder_vel(){
    _crc=0;
    _SendSignature(1,4);
    _SendIdTopic(7);
    _SendInt32((int32_t*)&Publisher_diff_encoder_vel.message.header.stamp.sec,1);
    _Sendcontinue();
    _SendUint32((uint32_t*)&Publisher_diff_encoder_vel.message.header.stamp.nanosec,1);
    _Sendcontinue();
    _SendString(&Publisher_diff_encoder_vel.message.header.frame_id,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_encoder_vel.message.vector.x,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_encoder_vel.message.vector.y,1);
    _Sendcontinue();
    _SendFloat32((float*)&Publisher_diff_encoder_vel.message.vector.z,1);
    _Sendstop();
    _Sendcrc();



















//get
void Xicro::Spin_node(){



    if(_serial->available() > 0){
        _serial->readBytes(_datain,1);

        
        switch (_state){
            case 0:
                if(_datain[0]==73){
                    _crcIn=0;
                    _state=1;
                }else{
                    _state=0;
                }
                break;
            case 1:
                if(_datain[0]==109){
                    _state=2;
                }else{
                    _state=0;
                }
                break;
            case 2:
                if(_datain[0]==64){
                    _state=3;
                }else{
                    _state=0;
                }
                break;
            case 3:
                if(_datain[0]==99){
                    _state=4;
                }else{
                    _state=0;
                }
                break;
            //strat protocol done
            case 4: // Signature
                q = _datain[0];
                w = _datain[0];
                q=q&0b11110000;
                q=q>>4;
                w=w&0b1111;
                if(q == _Idmcu && w==2){ // Onmode sub
                    _OnMode=2;
                    _state=5;  //check Idtopic
                }else if(q == _Idmcu && w==12){ // Onmode service client res
                    _OnMode=12;
                    _state=5; //check Idsrv
                }else if (q == _Idmcu && w==13){ // Onmode service server req
                    _OnMode=13;
                    _state=5; //check Idsrv
                }else if (q == _Idmcu && w==6){ // Onmode action feed
                    _OnMode=6;
                    _state=5; //check Idaction client
                }else if (q == _Idmcu && w==7){ // Onmode action res
                    _OnMode=7;
                    _state=5; //check Idactiion client
                }else if (q == _Idmcu && w==8){ // Onmode action server
                    _OnMode=8;
                    _state=5; //check Idactiion client
                }
                
                else{
                    _state=0;
                }
                break;
            case 5:
                if(_OnMode==2){
                    for (int i=0;i<sizeof(_Idtopic_sub);i++){
                        if(_Idtopic_sub[i]== _datain[0]){
                            _Indexdata=255;
                            _OnTopic=i;
                            _Onindex=0;
                            _state=6;
                        
                            break;
                        }else{
                            _state=0;
                        }
                    }
                }else if(_OnMode==12){
                    for (int i=0;i<sizeof(_Idsrv_client);i++){
                        if(_Idsrv_client[i]== _datain[0]){
                            _Indexdata=255;
                            _OnTopic=i;   // _OnTopic mean index of _topicType , _Srv_client_type[_OnTopic][]
                            _Onindex=0;   // _Onindex mean index of datatype in topic or srv , _Srv_client_type[][_Onindex] 
                            _state=6;
                            *(uint8_t*)_nonverify_srv_client_state[_OnTopic]=(uint8_t)0; // reset flag     
                            break;
                        }else{
                            _state=0;
                        }
                    }
                }else if(_OnMode==13){
                    for (int i=0;i<sizeof(_Idsrv_server);i++){
                        if(_Idsrv_server[i]== _datain[0]){
                            _Indexdata=255;
                            _OnTopic=i;   // _OnTopic mean index of _topicType , _Srv_client_type[_OnTopic][]
                            _Onindex=0;   // _Onindex mean index of datatype in topic or srv , _Srv_client_type[][_Onindex] 
                            _state=6;
                            *(uint8_t*)_nonverify_srv_server_state[_OnTopic]=(uint8_t)0; // reset flag     
                            break;
                        }else{
                            _state=0;
                        }
                    }
                }else if(_OnMode==6){ // action feedback
                    for (int i=0;i<sizeof(_Idaction_client);i++){
                        if(_Idaction_client[i]== _datain[0]){
                            _Indexdata=255;
                            _OnTopic=i;   // _OnTopic mean index of _topicType , _Srv_client_type[_OnTopic][]
                            _Onindex=0;   // _Onindex mean index of datatype in topic or srv , _Srv_client_type[][_Onindex] 
                            _state=6;
                            *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)0; // reset flag     
                            break;
                        }else{
                            _state=0;
                        }
                    }
                }else if(_OnMode==7){  // action res 
                    for (int i=0;i<sizeof(_Idaction_client);i++){
                        if(_Idaction_client[i]== _datain[0]){
                            _Indexdata=255;
                            _OnTopic=i;   // _OnTopic mean index of _topicType , _Srv_client_type[_OnTopic][]
                            _Onindex=0;   // _Onindex mean index of datatype in topic or srv , _Srv_client_type[][_Onindex] 
                            _state=6;
                            *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)0; // reset flag     
                            break;
                        }else{
                            _state=0;
                        }
                    }
                }else if(_OnMode==8){  // action server 
                    for (int i=0;i<sizeof(_Idaction_server);i++){
                        if(_Idaction_server[i]== _datain[0]){
                            _Indexdata=255;
                            _OnTopic=i;   // _OnTopic mean index of _topicType , _Srv_client_type[_OnTopic][]
                            _Onindex=0;   // _Onindex mean index of datatype in topic or srv , _Srv_client_type[][_Onindex] 
                            _state=6;
                            *(uint8_t*)_nonverify_action_server_state[_OnTopic]=(uint8_t)0; // reset flag     
                            break;
                        }else{
                            _state=0;
                        }
                    }
                }
                
                break;
            case 6:  //select type
                if(_OnMode == 2){
                    if(_datain[0]==_TopicType[_OnTopic][_Onindex]){
                        if(_datain[0]== 8  ||_datain[0]== 18 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _state=10;
                        }else if(_datain[0]== 9 ||_datain[0]== 19){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _statetogo=10;
                            _state=9;
                        }else if(_datain[0]==16 ||_datain[0]== 116){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  

                        }else if(_datain[0]==17 ||_datain[0]== 117){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==32 ||_datain[0]== 132){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;

                        }else if(_datain[0]==33 ||_datain[0]== 133){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab

                        }else if(_datain[0]==64||_datain[0]== 164){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;
                        }else if(_datain[0]==65 || _datain[0]==165){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==111){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  //get N byte
                        }else if(_datain[0]==112){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _bb=0;
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _statetogo=11;
                            _state=9; // verity N grab
                        
                        }else if(_datain[0]==242 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _state=99; // get 1 __STR_TYPE
                        }else if (_datain[0]==243){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _statetogo=99;
                            _state=9; // vreity N grab
                        }else if(_datain[0]==88){
                            _Indexdata=_Indexdata+1;
                            _bb=0;
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=50;  // 1 bool 
                        }else if(_datain[0]==89){
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=60;  // Check N bool 
                        }
                        
                        
                        else if(_datain[0]==126){
                            _state=90;
                        }
                        else{
                            _state=0;
                        }
                    
                        
                    }else{
                        _state=0;
                    }
                }else if(_OnMode == 12){   // Mode srv client recive response
                    if(_datain[0]==_Srv_client_res_Type[_OnTopic][_Onindex]){
                        if(_datain[0]== 8  ||_datain[0]== 18 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _state=10;
                        }else if(_datain[0]== 9 ||_datain[0]== 19){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _statetogo=10;
                            _state=9;
                        }else if(_datain[0]==16 ||_datain[0]== 116){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  

                        }else if(_datain[0]==17 ||_datain[0]== 117){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==32 ||_datain[0]== 132){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;

                        }else if(_datain[0]==33 ||_datain[0]== 133){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab

                        }else if(_datain[0]==64||_datain[0]== 164){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;
                        }else if(_datain[0]==65 || _datain[0]==165){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==111){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  //get N byte
                        }else if(_datain[0]==112){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _bb=0;
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _statetogo=11;
                            _state=9; // verity N grab
                        
                        }else if(_datain[0]==242 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _state=99; // get 1 __STR_TYPE
                        }else if (_datain[0]==243){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _statetogo=99;
                            _state=9; // vreity N grab
                        }else if(_datain[0]==88){
                            _Indexdata=_Indexdata+1;
                            _bb=0;
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=50;  // 1 bool 
                        }else if(_datain[0]==89){
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=60;  // Check N bool 
                        }
                        
                        
                        else if(_datain[0]==126){
                            _state=90;
                        }
                        else{
                            _state=0;
                        }
                    
                        
                    }else if(_datain[0]==252){  // seerver not response
                            *(uint8_t*)_nonverify_srv_client_state[_OnTopic]=(uint8_t)252;     
                            _state=111;      
                    }else if(_datain[0]==254){  // server return Emptyresponse
                        *(uint8_t*)_nonverify_srv_client_state[_OnTopic]=(uint8_t)254;     
                        _state=111;
                    }else if(_datain[0]==253){  // server return None or return timeout setup in yaml xicro
                        *(uint8_t*)_nonverify_srv_client_state[_OnTopic]=(uint8_t)253;     
                        _state=111;
                    }else{
                        _state=0;
                    }









                }else if(_OnMode == 13){   // Mode srv server req
                    if(_datain[0]==_Srv_server_req_Type[_OnTopic][_Onindex]){
                        if(_datain[0]== 8  ||_datain[0]== 18 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _state=10;
                        }else if(_datain[0]== 9 ||_datain[0]== 19){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _statetogo=10;
                            _state=9;
                        }else if(_datain[0]==16 ||_datain[0]== 116){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  

                        }else if(_datain[0]==17 ||_datain[0]== 117){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==32 ||_datain[0]== 132){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;

                        }else if(_datain[0]==33 ||_datain[0]== 133){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab

                        }else if(_datain[0]==64||_datain[0]== 164){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;
                        }else if(_datain[0]==65 || _datain[0]==165){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==111){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  //get N byte
                        }else if(_datain[0]==112){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _bb=0;
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _statetogo=11;
                            _state=9; // verity N grab
                        
                        }else if(_datain[0]==242 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _state=99; // get 1 __STR_TYPE
                        }else if (_datain[0]==243){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _statetogo=99;
                            _state=9; // vreity N grab
                        }else if(_datain[0]==88){
                            _Indexdata=_Indexdata+1;
                            _bb=0;
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=50;  // 1 bool 
                        }else if(_datain[0]==89){
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_srv_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=60;  // Check N bool 
                        }
                        
                        
                        else if(_datain[0]==126){
                            _state=90;
                        }
                        else{
                            _state=0;
                        }

                        
                    }else if(_datain[0]==252){  // seerver not response
                            *(uint8_t*)_nonverify_srv_server_state[_OnTopic]=(uint8_t)252;     
                            _state=111;      
                    }else if(_datain[0]==254){  // server return Emptyresponse
                        *(uint8_t*)_nonverify_srv_server_state[_OnTopic]=(uint8_t)254;     
                        _state=111;
                    }else if(_datain[0]==253){  // server return None or return timeout setup in yaml xicro
                        *(uint8_t*)_nonverify_srv_server_state[_OnTopic]=(uint8_t)253;     
                        _state=111;
                    }else{
                        _state=0;
                    }



                }else if(_OnMode == 6){   // Mode action client recive feedbacck 
                    if(_datain[0]==_Action_client_feed_Type[_OnTopic][_Onindex]){
                        if(_datain[0]== 8  ||_datain[0]== 18 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _state=10;
                        }else if(_datain[0]== 9 ||_datain[0]== 19){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _statetogo=10;
                            _state=9;
                        }else if(_datain[0]==16 ||_datain[0]== 116){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  

                        }else if(_datain[0]==17 ||_datain[0]== 117){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==32 ||_datain[0]== 132){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;

                        }else if(_datain[0]==33 ||_datain[0]== 133){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab

                        }else if(_datain[0]==64||_datain[0]== 164){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;
                        }else if(_datain[0]==65 || _datain[0]==165){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==111){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  //get N byte
                        }else if(_datain[0]==112){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _bb=0;
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _statetogo=11;
                            _state=9; // verity N grab
                        
                        }else if(_datain[0]==242 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _state=99; // get 1 __STR_TYPE
                        }else if (_datain[0]==243){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _statetogo=99;
                            _state=9; // vreity N grab
                        }else if(_datain[0]==88){
                            _Indexdata=_Indexdata+1;
                            _bb=0;
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=50;  // 1 bool 
                        }else if(_datain[0]==89){
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_feed[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=60;  // Check N bool 
                        }
                        
                        
                        else if(_datain[0]==126){
                            _state=90;
                        }
                        else{
                            _state=0;
                        }
                    
                        
                    }else if(_datain[0]==252){  // server not response
                            *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)252;     
                            _state=111;      
                    }else if(_datain[0]==254){  // server return Emptyresponse
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)254;     
                        _state=111;
                    }else if(_datain[0]==253){  // server return None 
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)253;     
                        _state=111;
                    }else if(_datain[0]==95){  // server goal accept
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)95;     
                        _state=111;
                    }else if(_datain[0]==96){  // server goal reject
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)96;     
                        _state=111;
                    }else if(_datain[0]==97){  // server time out in yaml
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)97;     
                        _state=111;
                    }else{
                        _state=0;
                    }









                }else if(_OnMode == 7){   // Mode action client recive res
                    if(_datain[0]==_Action_client_res_Type[_OnTopic][_Onindex]){
                        if(_datain[0]== 8  ||_datain[0]== 18 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _state=10;
                        }else if(_datain[0]== 9 ||_datain[0]== 19){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _statetogo=10;
                            _state=9;
                        }else if(_datain[0]==16 ||_datain[0]== 116){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  

                        }else if(_datain[0]==17 ||_datain[0]== 117){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==32 ||_datain[0]== 132){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;

                        }else if(_datain[0]==33 ||_datain[0]== 133){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab

                        }else if(_datain[0]==64||_datain[0]== 164){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;
                        }else if(_datain[0]==65 || _datain[0]==165){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==111){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  //get N byte
                        }else if(_datain[0]==112){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _bb=0;
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _statetogo=11;
                            _state=9; // verity N grab
                        
                        }else if(_datain[0]==242 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _state=99; // get 1 __STR_TYPE
                        }else if (_datain[0]==243){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _statetogo=99;
                            _state=9; // vreity N grab
                        }else if(_datain[0]==88){
                            _Indexdata=_Indexdata+1;
                            _bb=0;
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=50;  // 1 bool 
                        }else if(_datain[0]==89){
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_client_res[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=60;  // Check N bool 
                        }
                        
                        
                        else if(_datain[0]==126){
                            _state=90;
                        }
                        else{
                            _state=0;
                        }
                    
                        
                    }else if(_datain[0]==252){  // server not response
                            *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)252;     
                            _state=111;      
                    }else if(_datain[0]==254){  // server return Emptyresponse
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)254;     
                        _state=111;
                    }else if(_datain[0]==253){  // server return None 
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)253;     
                        _state=111;
                    }else if(_datain[0]==95){  // server goal accept
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)95;     
                        _state=111;
                    }else if(_datain[0]==96){  // server goal reject
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)96;     
                        _state=111;
                    }else if(_datain[0]==97){  // server time out in yaml
                        *(uint8_t*)_nonverify_action_client_state[_OnTopic]=(uint8_t)97;     
                        _state=111;
                    }else{
                        _state=0;
                    }









                }else if(_OnMode == 8){   // Mode action server req
                    if(_datain[0]==_Action_server_req_Type[_OnTopic][_Onindex]){
                        if(_datain[0]== 8  ||_datain[0]== 18 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _state=10;
                        }else if(_datain[0]== 9 ||_datain[0]== 19){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _statetogo=10;
                            _state=9;
                        }else if(_datain[0]==16 ||_datain[0]== 116){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  

                        }else if(_datain[0]==17 ||_datain[0]== 117){
                            _loop=0;
                            _loopTo=2;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==32 ||_datain[0]== 132){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;

                        }else if(_datain[0]==33 ||_datain[0]== 133){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab

                        }else if(_datain[0]==64||_datain[0]== 164){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;
                        }else if(_datain[0]==65 || _datain[0]==165){
                            _loop=0;
                            _loopTo=8;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _statetogo=11;
                            _state=9;  // verity N grab
                        }else if(_datain[0]==111){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _bb=0;
                            _state=11;  //get N byte
                        }else if(_datain[0]==112){
                            _loop=0;
                            _loopTo=4;
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _bb=0;
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _statetogo=11;
                            _state=9; // verity N grab
                        
                        }else if(_datain[0]==242 ){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _state=99; // get 1 __STR_TYPE
                        }else if (_datain[0]==243){
                            _Indexdata=_Indexdata+1;
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ontype=_datain[0];
                            _Ongrab=0;
                            _bufff="";
                            _statetogo=99;
                            _state=9; // vreity N grab
                        }else if(_datain[0]==88){
                            _Indexdata=_Indexdata+1;
                            _bb=0;
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=50;  // 1 bool 
                        }else if(_datain[0]==89){
                            _Indexdata=_Indexdata+1;
                            _Ontype=_datain[0];
                            _OngrabTo=_Nofdata_action_server_req[_OnTopic][_Indexdata];
                            _Ongrab=0;
                            _state=60;  // Check N bool 
                        }
                        
                        
                        else if(_datain[0]==126){
                            _state=90;
                        }
                        else{
                            _state=0;
                        }

                        
                    }else if(_datain[0]==252){  // server not response
                        *(uint8_t*)_nonverify_action_server_state[_OnTopic]=(uint8_t)252;     
                        _state=111;      
                    }else if(_datain[0]==254){  // server return Emptyresponse
                        *(uint8_t*)_nonverify_action_server_state[_OnTopic]=(uint8_t)254;     
                        _state=111;
                    }else if(_datain[0]==253){  // server return None or return timeout setup in yaml xicro
                        *(uint8_t*)_nonverify_action_server_state[_OnTopic]=(uint8_t)253;     
                        _state=111;
                    }else{
                        _state=0;
                    }



                }
                break;
            case 60:
                if(_datain[0]==_OngrabTo){
                    _state=61;
                }else{
                    _state=0;
                }
                break;
            case 61:
                for(int i=0;i<(8);i++){
                    if(i+ (_Ongrab*8)<_OngrabTo){
                        if(_OnMode == 2){
                            *(bool*)_nonverify[_OnTopic][_Indexdata][i+ (_Ongrab*8)] = (bool)(((uint8_t)_datain[0] >>i)&0x01);
                        }else if(_OnMode == 12){
                            *(bool*)_nonverify_srv_client_res[_OnTopic][_Indexdata][i+ (_Ongrab*8)] = (bool)(((uint8_t)_datain[0] >>i)&0x01);
                        }else if(_OnMode == 13){
                            *(bool*)_nonverify_srv_server_req[_OnTopic][_Indexdata][i+ (_Ongrab*8)] = (bool)(((uint8_t)_datain[0] >>i)&0x01);
                        }else if(_OnMode == 6){
                            *(bool*)_nonverify_action_client_feed[_OnTopic][_Indexdata][i+ (_Ongrab*8)] = (bool)(((uint8_t)_datain[0] >>i)&0x01);
                        }else if(_OnMode == 7){
                            *(bool*)_nonverify_action_client_res[_OnTopic][_Indexdata][i+ (_Ongrab*8)] = (bool)(((uint8_t)_datain[0] >>i)&0x01);
                        }else if(_OnMode == 8){
                            *(bool*)_nonverify_action_server_req[_OnTopic][_Indexdata][i+ (_Ongrab*8)] = (bool)(((uint8_t)_datain[0] >>i)&0x01);
                        }
                        
                    }
                    
                }
                _Ongrab=_Ongrab+1;
                if(_Ongrab==(uint8_t)ceil(_OngrabTo/8.00)){
                    
                    _state=222; // check main continue or stop
                }else{

                    _state=61; // grab more data
                }

                break;
            case 50:
                if(_datain[0]==250){ // continu true
                    _state=51;
                }else if(_datain[0]==47){ // continue false
                    _state=52;
                }else if(_datain[0]==254){ // stop true
                    _state=53;
                }else if(_datain[0]==127){ //stop false
                    _state=54;
                }else{
                    _state=0;
                }
                break;
            case 51: //confiirm true and continue
                if(_datain[0]==250){
                    if(_OnMode == 2){
                        *(bool*)_nonverify[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 12){
                        *(bool*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 13){
                        *(bool*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 6){
                        *(bool*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 7){
                        *(bool*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 8){
                        *(bool*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }
                    _Onindex=_Onindex+1;
                    _state=6;
                }else{
                    _state=0;
                }
                break;
            case 52: //confiirm false and continue
                if(_datain[0]==47){
                    if(_OnMode == 2){
                        *(bool*)_nonverify[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 12){
                        *(bool*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 13){
                        *(bool*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 6){
                        *(bool*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 7){
                        *(bool*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 8){
                        *(bool*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }
                    _Onindex=_Onindex+1;
                    _state=6;
                }else{
                    _state=0;
                }
                break;
            case 53: //confiirm true and stop
                if(_datain[0]==254){
                    if(_OnMode == 2){
                        *(bool*)_nonverify[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 12){
                        *(bool*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 13){
                        *(bool*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 6){
                        *(bool*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 7){
                        *(bool*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }else if(_OnMode == 8){
                        *(bool*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] = (bool)1;
                    }
                    _state=111;
                }else{
                    _state=0;
                }
                break;
            case 54: //confirm false and stop
                if(_datain[0]==127){
                    if(_OnMode == 2){
                        *(bool*)_nonverify[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 12){
                        *(bool*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 13){
                        *(bool*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 6){
                        *(bool*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 7){
                        *(bool*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }else if(_OnMode == 8){
                        *(bool*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] = (bool)0;
                    }
                    _state=111;
                }else{
                    _state=0;
                }
                break;




            case 9: // check N grab 
                if(_datain[0]==_OngrabTo){
                    _state=_statetogo;
                }else{
                    _state=0;
                }
                break;
            case 10: //uint8 and  int 8 
                if(_Ontype==8 || _Ontype==9){

                    if(_OnMode == 2){
                        *(uint8_t*)_nonverify[_OnTopic][_Indexdata][_Ongrab] = (uint8_t)_datain[0];
                    }else if(_OnMode == 12){
                        *(uint8_t*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] = (uint8_t)_datain[0];
                    }else if(_OnMode == 13){
                        *(uint8_t*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] = (uint8_t)_datain[0];
                    }else if(_OnMode == 6){
                        *(uint8_t*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] = (uint8_t)_datain[0];
                    }else if(_OnMode == 7){
                        *(uint8_t*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] = (uint8_t)_datain[0];
                    }else if(_OnMode == 8){
                        *(uint8_t*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] = (uint8_t)_datain[0];
                    }

                    
                }else if(_Ontype=18 || _Ontype==19){

                    if(_OnMode == 2){
                        *(int8_t*)_nonverify[_OnTopic][_Indexdata][_Ongrab] = (int8_t)_datain[0];
                    }else if(_OnMode == 12){
                        *(int8_t*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] = (int8_t)_datain[0];
                    }else if(_OnMode == 13){
                        *(int8_t*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] = (int8_t)_datain[0];
                    }else if(_OnMode == 6){
                        *(int8_t*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] = (int8_t)_datain[0];
                    }else if(_OnMode == 7){
                        *(int8_t*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] = (int8_t)_datain[0];
                    }else if(_OnMode == 8){
                        *(int8_t*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] = (int8_t)_datain[0];
                    }
                    
                }

                _Ongrab=_Ongrab+1;
                if(_Ongrab==_OngrabTo){
                    _state=222; // check main continue or stop
                }else{
                    _state=10; // grab more data
                }
                break;
   
            case 11: //n byte
                _loop=_loop+1;
                _bb= _bb<<8 ;
                _bb=_bb | _datain[0];
                if((_Ontype==16 || _Ontype==17 ) && _loop==_loopTo){

                    if(_OnMode == 2){
                        *(uint16_t*)_nonverify[_OnTopic][_Indexdata][_Ongrab] =   (uint16_t)_bb;
                    }else if(_OnMode == 12){
                        *(uint16_t*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] =   (uint16_t)_bb;
                    }else if(_OnMode == 13){
                        *(uint16_t*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] =   (uint16_t)_bb;
                    }else if(_OnMode == 6){
                        *(uint16_t*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] =   (uint16_t)_bb;
                    }else if(_OnMode == 7){
                        *(uint16_t*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] =   (uint16_t)_bb;
                    }else if(_OnMode == 8){
                        *(uint16_t*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] =   (uint16_t)_bb;
                    }

                    _state=222;
                }else if((_Ontype==116 || _Ontype==117 ) && _loop==_loopTo){

                    if(_OnMode == 2){
                        *(int16_t*)_nonverify[_OnTopic][_Indexdata][_Ongrab] =   (int16_t)_bb;
                    }else if(_OnMode == 12){
                        *(int16_t*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] =   (int16_t)_bb;
                    }else if(_OnMode == 13){
                        *(int16_t*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] =   (int16_t)_bb;
                    }else if(_OnMode == 6){
                        *(int16_t*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] =   (int16_t)_bb;
                    }else if(_OnMode == 7){
                        *(int16_t*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] =   (int16_t)_bb;
                    }else if(_OnMode == 8){
                        *(int16_t*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] =   (int16_t)_bb;
                    }
                    _state=222;
                }else if((_Ontype==32 || _Ontype==33 ) && _loop==_loopTo){

                    if(_OnMode == 2){
                        *(uint32_t*)_nonverify[_OnTopic][_Indexdata][_Ongrab] =   (uint32_t)_bb;
                    }else if(_OnMode == 12){
                        *(uint32_t*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] =   (uint32_t)_bb;
                    }else if(_OnMode == 13){
                        *(uint32_t*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] =   (uint32_t)_bb;
                    }else if(_OnMode == 6){
                        *(uint32_t*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] =   (uint32_t)_bb;
                    }else if(_OnMode == 7){
                        *(uint32_t*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] =   (uint32_t)_bb;
                    }else if(_OnMode == 8){
                        *(uint32_t*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] =   (uint32_t)_bb;
                    }

                    _state=222;
                }else if((_Ontype==132 || _Ontype==133 ) && _loop==_loopTo){

                    if(_OnMode == 2){
                        *(int32_t*)_nonverify[_OnTopic][_Indexdata][_Ongrab] =   (int32_t)_bb;
                    }else if(_OnMode == 12){
                        *(int32_t*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] =   (int32_t)_bb;
                    }else if(_OnMode == 13){
                        *(int32_t*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] =   (int32_t)_bb;
                    }else if(_OnMode == 6){
                        *(int32_t*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] =   (int32_t)_bb;
                    }else if(_OnMode == 7){
                        *(int32_t*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] =   (int32_t)_bb;
                    }else if(_OnMode == 8){
                        *(int32_t*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] =   (int32_t)_bb;
                    }

                    _state=222;
                }else if((_Ontype==64 || _Ontype==65 ) && _loop==_loopTo){

                    if(_OnMode == 2){
                        *(uint64_t*)_nonverify[_OnTopic][_Indexdata][_Ongrab] =   (uint64_t)_bb;
                    }else if(_OnMode == 12){
                        *(uint64_t*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] =   (uint64_t)_bb;
                    }else if(_OnMode == 13){
                        *(uint64_t*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] =   (uint64_t)_bb;
                    }else if(_OnMode == 6){
                        *(uint64_t*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] =   (uint64_t)_bb;
                    }else if(_OnMode == 7){
                        *(uint64_t*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] =   (uint64_t)_bb;
                    }else if(_OnMode == 8){
                        *(uint64_t*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] =   (uint64_t)_bb;
                    }

                    _state=222;
                }else if((_Ontype==164 || _Ontype==165 ) && _loop==_loopTo){

                    if(_OnMode == 2){
                        *(int64_t*)_nonverify[_OnTopic][_Indexdata][_Ongrab] =   (int64_t)_bb;
                    }else if(_OnMode == 12){
                        *(int64_t*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] =   (int64_t)_bb;
                    }else if(_OnMode == 13){
                        *(int64_t*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] =   (int64_t)_bb;
                    }else if(_OnMode == 6){
                        *(int64_t*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] =   (int64_t)_bb;
                    }else if(_OnMode == 7){
                        *(int64_t*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] =   (int64_t)_bb;
                    }else if(_OnMode == 8){
                        *(int64_t*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] =   (int64_t)_bb;
                    }
                    
                    _state=222;
                }else if((_Ontype==111 || _Ontype==112 ) && _loop==_loopTo){
                    uint8_t bu[4];
                    bu[0] = (_bb >> 0)  & 0xFF;
                    bu[1] = (_bb >> 8)  & 0xFF;
                    bu[2] = (_bb >> 16) & 0xFF;
                    bu[3] = (_bb >> 24) & 0xFF;

                    if(_OnMode == 2){
                        memcpy (_nonverify[_OnTopic][_Indexdata][_Ongrab], bu, 4);
                    }else if(_OnMode == 12){
                        memcpy (_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab], bu, 4);
                    }else if(_OnMode == 13){
                        memcpy (_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab], bu, 4);
                    }else if(_OnMode == 6){
                        memcpy (_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab], bu, 4);
                    }else if(_OnMode == 13){
                        memcpy (_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab], bu, 4);
                    }else if(_OnMode == 8){
                        memcpy (_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab], bu, 4);
                    }
                    
                    _state=222;
                }

                if( _loop==_loopTo){
                    _bb= 0;
                    _loop=0;
                    _state=11;  // get more N data
                    _Ongrab=_Ongrab+1;
                }
                if(_Ongrab==_OngrabTo ){
                    _state=222; // check main continue or stop
                }

                break;
            case 90:
                if(_datain[0]==126){
                    _state=111;  //checksum
                        
                }else{
                    _state=0;
                }
            
                break;
            case 111:
                // __SEND_UART(_crcIn);
                if(_crcIn==_datain[0]){
                    if(_OnMode==2){
                        for (int i=0;i<_Totalvar[_OnTopic];i++){
                            for(int k=0;k<_Nofdata[_OnTopic][i];k++){
                                if(_TopicType[_OnTopic][i]== 8 || _TopicType[_OnTopic][i]== 9 ){
                                    *(uint8_t*)_verify[_OnTopic][i][k]=*(uint8_t*)_nonverify[_OnTopic][i][k];
                                }else if ( _TopicType[_OnTopic][i]== 88 || _TopicType[_OnTopic][i]== 89){
                                    *(bool*)_verify[_OnTopic][i][k]=*(bool*)_nonverify[_OnTopic][i][k];
                                }else if (_TopicType[_OnTopic][i]== 18 || _TopicType[_OnTopic][i]== 19){
                                    *(int8_t*)_verify[_OnTopic][i][k]=*(int8_t*)_nonverify[_OnTopic][i][k];
                                }else if (_TopicType[_OnTopic][i]== 16 || _TopicType[_OnTopic][i]== 17){
                                    *(uint16_t*)_verify[_OnTopic][i][k]=*(uint16_t*)_nonverify[_OnTopic][i][k];
                                }else if (_TopicType[_OnTopic][i]== 116 || _TopicType[_OnTopic][i]== 117){
                                    *(int16_t*)_verify[_OnTopic][i][k]=*(int16_t*)_nonverify[_OnTopic][i][k];
                                }else if (_TopicType[_OnTopic][i]== 32 || _TopicType[_OnTopic][i]== 33 ){
                                    *(uint32_t*)_verify[_OnTopic][i][k]=*(uint32_t*)_nonverify[_OnTopic][i][k];
                                }else if (_TopicType[_OnTopic][i]== 132 || _TopicType[_OnTopic][i]== 133 ){
                                    *(int32_t*)_verify[_OnTopic][i][k]=*(int32_t*)_nonverify[_OnTopic][i][k];
                                }else if (_TopicType[_OnTopic][i]== 64 || _TopicType[_OnTopic][i]== 65 ){
                                    *(uint64_t*)_verify[_OnTopic][i][k]=*(uint64_t*)_nonverify[_OnTopic][i][k];
                                }else if (_TopicType[_OnTopic][i]== 164 || _TopicType[_OnTopic][i]== 165 ){
                                    *(int64_t*)_verify[_OnTopic][i][k]=*(int64_t*)_nonverify[_OnTopic][i][k];
                                }else if (_TopicType[_OnTopic][i]== 111 || _TopicType[_OnTopic][i]== 112){
                                    *(float*)_verify[_OnTopic][i][k]=*(float*)_nonverify[_OnTopic][i][k];
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 242 || _TopicType[_OnTopic][i]== 243){
                                    *(__STR_TYPE*)_verify[_OnTopic][i][k]=*(__STR_TYPE*)_nonverify[_OnTopic][i][k];  
                                }
                            }
                        }
                    }else if(_OnMode==12){
                        for (int i=0;i<_Totalvar_srv_client_res[_OnTopic];i++){
                            for(int k=0;k<_Nofdata_srv_client_res[_OnTopic][i];k++){
                                if(_Srv_client_res_Type[_OnTopic][i]== 8 || _Srv_client_res_Type[_OnTopic][i]== 9 ){
                                    *(uint8_t*)_verify_srv_client_res[_OnTopic][i][k]=*(uint8_t*)_nonverify_srv_client_res[_OnTopic][i][k];
                                }else if ( _Srv_client_res_Type[_OnTopic][i]== 88 || _Srv_client_res_Type[_OnTopic][i]== 89){
                                    *(bool*)_verify_srv_client_res[_OnTopic][i][k]=*(bool*)_nonverify_srv_client_res[_OnTopic][i][k];
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 18 || _Srv_client_res_Type[_OnTopic][i]== 19){
                                    *(int8_t*)_verify_srv_client_res[_OnTopic][i][k]=*(int8_t*)_nonverify_srv_client_res[_OnTopic][i][k];
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 16 || _Srv_client_res_Type[_OnTopic][i]== 17){
                                    *(uint16_t*)_verify_srv_client_res[_OnTopic][i][k]=*(uint16_t*)_nonverify_srv_client_res[_OnTopic][i][k];
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 116 || _Srv_client_res_Type[_OnTopic][i]== 117){
                                    *(int16_t*)_verify_srv_client_res[_OnTopic][i][k]=*(int16_t*)_nonverify_srv_client_res[_OnTopic][i][k];
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 32 || _Srv_client_res_Type[_OnTopic][i]== 33 ){
                                    *(uint32_t*)_verify_srv_client_res[_OnTopic][i][k]=*(uint32_t*)_nonverify_srv_client_res[_OnTopic][i][k];   
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 132 || _Srv_client_res_Type[_OnTopic][i]== 133 ){
                                    *(int32_t*)_verify_srv_client_res[_OnTopic][i][k]=*(int32_t*)_nonverify_srv_client_res[_OnTopic][i][k];
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 64 || _Srv_client_res_Type[_OnTopic][i]== 65 ){
                                    *(uint64_t*)_verify_srv_client_res[_OnTopic][i][k]=*(uint64_t*)_nonverify_srv_client_res[_OnTopic][i][k];
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 164 || _Srv_client_res_Type[_OnTopic][i]== 165 ){
                                    *(int64_t*)_verify_srv_client_res[_OnTopic][i][k]=*(int64_t*)_nonverify_srv_client_res[_OnTopic][i][k];                           
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 111 || _Srv_client_res_Type[_OnTopic][i]== 112){
                                    *(float*)_verify_srv_client_res[_OnTopic][i][k]=*(float*)_nonverify_srv_client_res[_OnTopic][i][k];               
                                }else if (_Srv_client_res_Type[_OnTopic][i]== 242 || _Srv_client_res_Type[_OnTopic][i]== 243){
                                    *(__STR_TYPE*)_verify_srv_client_res[_OnTopic][i][k]=*(__STR_TYPE*)_nonverify_srv_client_res[_OnTopic][i][k];
                                }
                            }

                        }
                        if(*(uint8_t*)_nonverify_srv_client_state[_OnTopic]!=0){
                            *(uint8_t*)_verify_srv_client_state[_OnTopic]=*(uint8_t*)_nonverify_srv_client_state[_OnTopic];
                        }else{
                            *(uint8_t*)_verify_srv_client_state[_OnTopic]=(uint8_t)2;
                        } 
                    }else if(_OnMode==13){
                        for (int i=0;i<_Totalvar_srv_server_req[_OnTopic];i++){
                            for(int k=0;k<_Nofdata_srv_server_req[_OnTopic][i];k++){
                                if(_Srv_server_req_Type[_OnTopic][i]== 8 || _Srv_server_req_Type[_OnTopic][i]== 9 ){
                                    *(uint8_t*)_verify_srv_server_req[_OnTopic][i][k]=*(uint8_t*)_nonverify_srv_server_req[_OnTopic][i][k];
                                }else if ( _Srv_server_req_Type[_OnTopic][i]== 88 || _Srv_server_req_Type[_OnTopic][i]== 89){
                                    *(bool*)_verify_srv_server_req[_OnTopic][i][k]=*(bool*)_nonverify_srv_server_req[_OnTopic][i][k];
                                }else if (_Srv_server_req_Type[_OnTopic][i]== 18 || _Srv_server_req_Type[_OnTopic][i]== 19){
                                    *(int8_t*)_verify_srv_server_req[_OnTopic][i][k]=*(int8_t*)_nonverify_srv_server_req[_OnTopic][i][k];
                                }else if (_Srv_server_req_Type[_OnTopic][i]== 16 || _Srv_server_req_Type[_OnTopic][i]== 17){
                                    *(uint16_t*)_verify_srv_server_req[_OnTopic][i][k]=*(uint16_t*)_nonverify_srv_server_req[_OnTopic][i][k];
                                }else if (_Srv_server_req_Type[_OnTopic][i]== 116 || _Srv_server_req_Type[_OnTopic][i]== 117){
                                    *(int16_t*)_verify_srv_server_req[_OnTopic][i][k]=*(int16_t*)_nonverify_srv_server_req[_OnTopic][i][k];
                                }else if (_Srv_server_req_Type[_OnTopic][i]== 32 || _Srv_server_req_Type[_OnTopic][i]== 33 ){
                                    *(uint32_t*)_verify_srv_server_req[_OnTopic][i][k]=*(uint32_t*)_nonverify_srv_server_req[_OnTopic][i][k];   
                                }else if (_Srv_server_req_Type[_OnTopic][i]== 132 || _Srv_server_req_Type[_OnTopic][i]== 133 ){
                                    *(int32_t*)_verify_srv_server_req[_OnTopic][i][k]=*(int32_t*)_nonverify_srv_server_req[_OnTopic][i][k];
                                }else if (_Srv_server_req_Type[_OnTopic][i]== 64 || _Srv_server_req_Type[_OnTopic][i]== 65 ){
                                    *(uint64_t*)_verify_srv_server_req[_OnTopic][i][k]=*(uint64_t*)_nonverify_srv_server_req[_OnTopic][i][k];
                                }else if (_Srv_server_req_Type[_OnTopic][i]== 164 || _Srv_server_req_Type[_OnTopic][i]== 165 ){
                                    *(int64_t*)_verify_srv_server_req[_OnTopic][i][k]=*(int64_t*)_nonverify_srv_server_req[_OnTopic][i][k];                           
                                }else if (_Srv_server_req_Type[_OnTopic][i]== 111 || _Srv_server_req_Type[_OnTopic][i]== 112){
                                    *(float*)_verify_srv_server_req[_OnTopic][i][k]=*(float*)_nonverify_srv_server_req[_OnTopic][i][k];               
                                }else if (_Srv_server_req_Type[_OnTopic][i]== 242 || _Srv_server_req_Type[_OnTopic][i]== 243){
                                    *(__STR_TYPE*)_verify_srv_server_req[_OnTopic][i][k]=*(__STR_TYPE*)_nonverify_srv_server_req[_OnTopic][i][k];
                                }
                            }

                        }
                        if(*(uint8_t*)_nonverify_srv_server_state[_OnTopic]!=0){
                            *(uint8_t*)_verify_srv_server_state[_OnTopic]=*(uint8_t*)_nonverify_srv_server_state[_OnTopic];
                        }else{
                            *(uint8_t*)_verify_srv_server_state[_OnTopic]=(uint8_t)2;
                        } 
                        ((void (*)(void))_service_server[_OnTopic])();
                    }else if(_OnMode==6){
                        for (int i=0;i<_Totalvar_action_client_feed[_OnTopic];i++){
                            for(int k=0;k<_Nofdata_action_client_feed[_OnTopic][i];k++){
                                if(_Action_client_feed_Type[_OnTopic][i]== 8 || _Action_client_feed_Type[_OnTopic][i]== 9 ){
                                    *(uint8_t*)_verify_action_client_feed[_OnTopic][i][k]=*(uint8_t*)_nonverify_action_client_feed[_OnTopic][i][k];
                                }else if ( _Action_client_feed_Type[_OnTopic][i]== 88 || _Action_client_feed_Type[_OnTopic][i]== 89){
                                    *(bool*)_verify_action_client_feed[_OnTopic][i][k]=*(bool*)_nonverify_action_client_feed[_OnTopic][i][k];
                                }else if (_Action_client_feed_Type[_OnTopic][i]== 18 || _Action_client_feed_Type[_OnTopic][i]== 19){
                                    *(int8_t*)_verify_action_client_feed[_OnTopic][i][k]=*(int8_t*)_nonverify_action_client_feed[_OnTopic][i][k];
                                }else if (_Action_client_feed_Type[_OnTopic][i]== 16 || _Action_client_feed_Type[_OnTopic][i]== 17){
                                    *(uint16_t*)_verify_action_client_feed[_OnTopic][i][k]=*(uint16_t*)_nonverify_action_client_feed[_OnTopic][i][k];
                                }else if (_Action_client_feed_Type[_OnTopic][i]== 116 || _Action_client_feed_Type[_OnTopic][i]== 117){
                                    *(int16_t*)_verify_action_client_feed[_OnTopic][i][k]=*(int16_t*)_nonverify_action_client_feed[_OnTopic][i][k];
                                }else if (_Action_client_feed_Type[_OnTopic][i]== 32 || _Action_client_feed_Type[_OnTopic][i]== 33 ){
                                    *(uint32_t*)_verify_action_client_feed[_OnTopic][i][k]=*(uint32_t*)_nonverify_action_client_feed[_OnTopic][i][k];   
                                }else if (_Action_client_feed_Type[_OnTopic][i]== 132 || _Action_client_feed_Type[_OnTopic][i]== 133 ){
                                    *(int32_t*)_verify_action_client_feed[_OnTopic][i][k]=*(int32_t*)_nonverify_action_client_feed[_OnTopic][i][k];
                                }else if (_Action_client_feed_Type[_OnTopic][i]== 64 || _Action_client_feed_Type[_OnTopic][i]== 65 ){
                                    *(uint64_t*)_verify_action_client_feed[_OnTopic][i][k]=*(uint64_t*)_nonverify_action_client_feed[_OnTopic][i][k];
                                }else if (_Action_client_feed_Type[_OnTopic][i]== 164 || _Action_client_feed_Type[_OnTopic][i]== 165 ){
                                    *(int64_t*)_verify_action_client_feed[_OnTopic][i][k]=*(int64_t*)_nonverify_action_client_feed[_OnTopic][i][k];                           
                                }else if (_Action_client_feed_Type[_OnTopic][i]== 111 || _Action_client_feed_Type[_OnTopic][i]== 112){
                                    *(float*)_verify_action_client_feed[_OnTopic][i][k]=*(float*)_nonverify_action_client_feed[_OnTopic][i][k];               
                                }else if (_Action_client_feed_Type[_OnTopic][i]== 242 || _Action_client_feed_Type[_OnTopic][i]== 243){
                                    *(__STR_TYPE*)_verify_action_client_feed[_OnTopic][i][k]=*(__STR_TYPE*)_nonverify_action_client_feed[_OnTopic][i][k];
                                }
                            }

                        }
                        if(*(uint8_t*)_nonverify_action_client_state[_OnTopic]!=0){
                            *(uint8_t*)_verify_action_client_state[_OnTopic]=*(uint8_t*)_nonverify_action_client_state[_OnTopic];
                        }else{
                            *(uint8_t*)_verify_action_client_state[_OnTopic]=(uint8_t)4;
                        } 
                    }else if(_OnMode==7){
                        for (int i=0;i<_Totalvar_action_client_res[_OnTopic];i++){
                            for(int k=0;k<_Nofdata_action_client_res[_OnTopic][i];k++){
                                if(_Action_client_res_Type[_OnTopic][i]== 8 || _Action_client_res_Type[_OnTopic][i]== 9 ){
                                    *(uint8_t*)_verify_action_client_res[_OnTopic][i][k]=*(uint8_t*)_nonverify_action_client_res[_OnTopic][i][k];
                                }else if ( _Action_client_res_Type[_OnTopic][i]== 88 || _Action_client_res_Type[_OnTopic][i]== 89){
                                    *(bool*)_verify_action_client_res[_OnTopic][i][k]=*(bool*)_nonverify_action_client_res[_OnTopic][i][k];
                                }else if (_Action_client_res_Type[_OnTopic][i]== 18 || _Action_client_res_Type[_OnTopic][i]== 19){
                                    *(int8_t*)_verify_action_client_res[_OnTopic][i][k]=*(int8_t*)_nonverify_action_client_res[_OnTopic][i][k];
                                }else if (_Action_client_res_Type[_OnTopic][i]== 16 || _Action_client_res_Type[_OnTopic][i]== 17){
                                    *(uint16_t*)_verify_action_client_res[_OnTopic][i][k]=*(uint16_t*)_nonverify_action_client_res[_OnTopic][i][k];
                                }else if (_Action_client_res_Type[_OnTopic][i]== 116 || _Action_client_res_Type[_OnTopic][i]== 117){
                                    *(int16_t*)_verify_action_client_res[_OnTopic][i][k]=*(int16_t*)_nonverify_action_client_res[_OnTopic][i][k];
                                }else if (_Action_client_res_Type[_OnTopic][i]== 32 || _Action_client_res_Type[_OnTopic][i]== 33 ){
                                    *(uint32_t*)_verify_action_client_res[_OnTopic][i][k]=*(uint32_t*)_nonverify_action_client_res[_OnTopic][i][k];   
                                }else if (_Action_client_res_Type[_OnTopic][i]== 132 || _Action_client_res_Type[_OnTopic][i]== 133 ){
                                    *(int32_t*)_verify_action_client_res[_OnTopic][i][k]=*(int32_t*)_nonverify_action_client_res[_OnTopic][i][k];
                                }else if (_Action_client_res_Type[_OnTopic][i]== 64 || _Action_client_res_Type[_OnTopic][i]== 65 ){
                                    *(uint64_t*)_verify_action_client_res[_OnTopic][i][k]=*(uint64_t*)_nonverify_action_client_res[_OnTopic][i][k];
                                }else if (_Action_client_res_Type[_OnTopic][i]== 164 || _Action_client_res_Type[_OnTopic][i]== 165 ){
                                    *(int64_t*)_verify_action_client_res[_OnTopic][i][k]=*(int64_t*)_nonverify_action_client_res[_OnTopic][i][k];                           
                                }else if (_Action_client_res_Type[_OnTopic][i]== 111 || _Action_client_res_Type[_OnTopic][i]== 112){
                                    *(float*)_verify_action_client_res[_OnTopic][i][k]=*(float*)_nonverify_action_client_res[_OnTopic][i][k];               
                                }else if (_Action_client_res_Type[_OnTopic][i]== 242 || _Action_client_res_Type[_OnTopic][i]== 243){
                                    *(__STR_TYPE*)_verify_action_client_res[_OnTopic][i][k]=*(__STR_TYPE*)_nonverify_action_client_res[_OnTopic][i][k];
                                }
                            }

                        }
                        if(*(uint8_t*)_nonverify_action_client_state[_OnTopic]!=0){
                            *(uint8_t*)_verify_action_client_state[_OnTopic]=*(uint8_t*)_nonverify_action_client_state[_OnTopic];
                        }else{
                            *(uint8_t*)_verify_action_client_state[_OnTopic]=(uint8_t)5;
                        } 
                    }else if(_OnMode==8){
                        for (int i=0;i<_Totalvar_action_server_req[_OnTopic];i++){
                            for(int k=0;k<_Nofdata_action_server_req[_OnTopic][i];k++){
                                if(_Action_server_req_Type[_OnTopic][i]== 8 || _Action_server_req_Type[_OnTopic][i]== 9 ){
                                    *(uint8_t*)_verify_action_server_req[_OnTopic][i][k]=*(uint8_t*)_nonverify_action_server_req[_OnTopic][i][k];
                                }else if ( _Action_server_req_Type[_OnTopic][i]== 88 || _Action_server_req_Type[_OnTopic][i]== 89){
                                    *(bool*)_verify_action_server_req[_OnTopic][i][k]=*(bool*)_nonverify_action_server_req[_OnTopic][i][k];
                                }else if (_Action_server_req_Type[_OnTopic][i]== 18 || _Action_server_req_Type[_OnTopic][i]== 19){
                                    *(int8_t*)_verify_action_server_req[_OnTopic][i][k]=*(int8_t*)_nonverify_action_server_req[_OnTopic][i][k];
                                }else if (_Action_server_req_Type[_OnTopic][i]== 16 || _Action_server_req_Type[_OnTopic][i]== 17){
                                    *(uint16_t*)_verify_action_server_req[_OnTopic][i][k]=*(uint16_t*)_nonverify_action_server_req[_OnTopic][i][k];
                                }else if (_Action_server_req_Type[_OnTopic][i]== 116 || _Action_server_req_Type[_OnTopic][i]== 117){
                                    *(int16_t*)_verify_action_server_req[_OnTopic][i][k]=*(int16_t*)_nonverify_action_server_req[_OnTopic][i][k];
                                }else if (_Action_server_req_Type[_OnTopic][i]== 32 || _Action_server_req_Type[_OnTopic][i]== 33 ){
                                    *(uint32_t*)_verify_action_server_req[_OnTopic][i][k]=*(uint32_t*)_nonverify_action_server_req[_OnTopic][i][k];   
                                }else if (_Action_server_req_Type[_OnTopic][i]== 132 || _Action_server_req_Type[_OnTopic][i]== 133 ){
                                    *(int32_t*)_verify_action_server_req[_OnTopic][i][k]=*(int32_t*)_nonverify_action_server_req[_OnTopic][i][k];
                                }else if (_Action_server_req_Type[_OnTopic][i]== 64 || _Action_server_req_Type[_OnTopic][i]== 65 ){
                                    *(uint64_t*)_verify_action_server_req[_OnTopic][i][k]=*(uint64_t*)_nonverify_action_server_req[_OnTopic][i][k];
                                }else if (_Action_server_req_Type[_OnTopic][i]== 164 || _Action_server_req_Type[_OnTopic][i]== 165 ){
                                    *(int64_t*)_verify_action_server_req[_OnTopic][i][k]=*(int64_t*)_nonverify_action_server_req[_OnTopic][i][k];                           
                                }else if (_Action_server_req_Type[_OnTopic][i]== 111 || _Action_server_req_Type[_OnTopic][i]== 112){
                                    *(float*)_verify_action_server_req[_OnTopic][i][k]=*(float*)_nonverify_action_server_req[_OnTopic][i][k];               
                                }else if (_Action_server_req_Type[_OnTopic][i]== 242 || _Action_server_req_Type[_OnTopic][i]== 243){
                                    *(__STR_TYPE*)_verify_action_server_req[_OnTopic][i][k]=*(__STR_TYPE*)_nonverify_action_server_req[_OnTopic][i][k];
                                }
                            }

                        }
                        if(*(uint8_t*)_nonverify_action_server_state[_OnTopic]!=0){
                            *(uint8_t*)_verify_action_server_state[_OnTopic]=*(uint8_t*)_nonverify_action_server_state[_OnTopic];
                        }else{
                            *(uint8_t*)_verify_action_server_state[_OnTopic]=(uint8_t)2;
                        } 
                        ((void (*)(void))_action_server[_OnTopic])();
                    }



                }
                _state=0;
                break;
            case 99: //__STR_TYPE

                if(_datain[0]==42){
                    _state=100;
                }else{
                    _bufff=_bufff+char(_datain[0]);
                    _state=99;
                }
                break;
            case 100:
                if(_datain[0]==126){

                    if(_OnMode == 2){
                        *(__STR_TYPE*)_nonverify[_OnTopic][_Indexdata][_Ongrab] =   (__STR_TYPE)_bufff;
                    }else if(_OnMode == 12){
                        *(__STR_TYPE*)_nonverify_srv_client_res[_OnTopic][_Indexdata][_Ongrab] =   (__STR_TYPE)_bufff;                        
                    }else if(_OnMode == 13){
                        *(__STR_TYPE*)_nonverify_srv_server_req[_OnTopic][_Indexdata][_Ongrab] =   (__STR_TYPE)_bufff;                        
                    }else if(_OnMode == 6){
                        *(__STR_TYPE*)_nonverify_action_client_feed[_OnTopic][_Indexdata][_Ongrab] =   (__STR_TYPE)_bufff;                        
                    }else if(_OnMode == 7){
                        *(__STR_TYPE*)_nonverify_action_client_res[_OnTopic][_Indexdata][_Ongrab] =   (__STR_TYPE)_bufff;                        
                    }else if(_OnMode == 8){
                        *(__STR_TYPE*)_nonverify_action_server_req[_OnTopic][_Indexdata][_Ongrab] =   (__STR_TYPE)_bufff;                        
                    }
                    
                    _Ongrab=_Ongrab+1;
                    if(_Ongrab==_OngrabTo){
                        _state=222; // check main continue or stop
                    }else{
                        _state=99;  // get more __STR_TYPE
                    }
                }else{
                    _state=0;
                }

                break;

            case 222: // main check continue or stop
                if(_datain[0]==42){
                    _state=223; // to ckeck confirm continue
                }else if(_datain[0]==126){
                    _state=225; // to ckeck confirm stop
                }else{
                    _state=0;
                }
                break;
            case 223:
                if(_datain[0]==42){ // continue done
                    _state=6; // check data type 
                    _Onindex=_Onindex+1;
                }else{
                    _state=0;
                }
                break;

            case 225:
                 if(_datain[0]==126){ // stop done
                    _state=111; //crc check to Verify data
                }else{
                    _state=0;
                }
                break;
        }
        _crcIn=_getcrc(_crcIn,&_datain[0],1); 


    }
}