#define RADAR_POS_SELF -1

#define RADAR_POS_CENTER -2
#define RADAR_MAP_TYPE_SAVED 'S'
#define RADAR_MAP_TYPE_SINGLE 'D'
#define RADAR_MAP_TMP -1
#define RADAR_LENTH_TYPE_MAX 'X'
#define RADAR_LENTH_TYPE_MIN 'I'
#define RADAR_LENTH_TYPE_NORMAL 'L'
#define RADAR_LENTH_TYPE_NON 'N'
#define RADAR_LENTH_TYPE_FAR 'F'
#define RADAR_RANGE_MAX  119.9
#define RADAR_RANGE_MIN  -119.6
#define RADAR_TIME  105

#define Degree_range 10
#define WIN_SIZE 800.0

#define LINE_THRESHOLD 0.04363323129 //5degree
#define LINE_THRESHOLD_RANGE 0.34906585038 //40degree
#define LINE_OVER_THRESHOLD 2.79252680321


#define Weel_step2metter_Constant ( 42.46) //( 10000 / (250 * M_PI))
#define Weel_step2metter_Parameter (1.000) //1.02315228880397 
#define Weel_step2metter (  Weel_step2metter_Parameter * Weel_step2metter_Constant )

#define Weel_step2degree_Constant  ( 39550/360.0 ) //((1800/360 ) *  ( 10000/ (250*M_PI) ) )
#define Weel_step2degree_Parameter (1.005) //1.0610 1.05692970136262 1.05704572507684
#define Weel_step2degree (Weel_step2degree_Parameter * Weel_step2degree_Constant ) ///1043
#define stayparameter ( 6.2 )

#define CAR_WELL_L 297.977 



#define OrientMotorCom "/dev/hl340"
