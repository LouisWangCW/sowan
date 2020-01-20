#ifndef __CAR_PARA_DEF__
#define __CAR_PARA_DEF__

#define Degree_range 10
#define WIN_SIZE 800.0


#define Weel_plus_per_mm ( 337.858 ) // step/mm
#define Weel_plus_per_meter (Weel_plus_per_mm * 1000.0)

#define Weel_plus_per_deg  ( 883.222 ) //  317962/360.0 => step/deg 883.222 

#define Weel_plus_per_rad (50604.89297) /// Weel_plus_per_deg * 180 / PI

#define Weel_spd_meter_sec_2_RPM ( Weel_plus_per_meter * 60.0 / 10000.0 )
#define Weel_spd_rad_sec_2_RPM ( Weel_plus_per_rad * 60.0 / 10000.0 )

#define Weel_plus_per_circle 10000

#define CAR_WELL_to_Weel_L 298
#define CAR_WELL_to_Weel_L_meter 0.298

#define OrientMotorCom "/dev/hl340"

#define HOME_ROOT "/home/ubuntu/catkin_ws/src/sowan"
#define VOICE_PLAY_CMD "paplay -p "

#endif