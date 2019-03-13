#include <XPLMGraphics.h>
#include <XPLMDisplay.h>

#include "XPLMUtilities.h"
#include "XPLMScenery.h"
#include "XPLMInstance.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"

#include <XPLMDataAccess.h>
#include <iostream>
#include <cstdio>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>


#if IBM
	#include <windows.h>
#endif
#if LIN
	#include <GL/gl.h>
#elif __GNUC__
	#include <OpenGL/gl.h>
#else
	#include <GL/gl.h>
#endif

#ifndef XPLM300
	#error This is made to be compiled against the XPLM300 SDK
#endif

// variables needed for the simulator
float dt = -1;//time interval: negative to cycles
float pi = 3.14159265359;
float deg2rad = pi/180;
float rad2deg = 1/deg2rad;
float mt2ft = 0.3048;

// socket properties definition
const char* Host = "127.0.0.1";
int socketPort = 9016;

char InstructionsText[50][200] = {
"  Network options",
"  Host address:",
"  Port:",
"  FDM internal/external",
""
};

// initialize widget
XPWidgetID	ExternalFDMWidget = NULL;
XPWidgetID	InstructionsWindow = NULL;
XPWidgetID	FDMDeactivatorTextWidget[50] = {NULL};

void InstructionsMenuHandler(void *, void *);
void CreateWidget(int x1, int y1, int w, int h);
int InstructionsHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, long  inParam1, long  inParam2);
int gMenuItem;

// initialize the datarefs
XPLMDataRef OverrideFDMDataRef = NULL;
XPLMDataRef OverrideJoystickDataRef = NULL;
XPLMDataRef AvionicsOn = NULL;

//XPLMDataRef FlightModelReactivateDataRef = NULL;

XPLMDataRef aircraft_xLocalDataRef = NULL;
XPLMDataRef aircraft_yLocalDataRef = NULL;
XPLMDataRef aircraft_zLocalDataRef = NULL;
XPLMDataRef aircraft_indicatedAltitudeDataRef = NULL;
XPLMDataRef aircraft_verticalSpeedDataRef = NULL;
XPLMDataRef aircraft_lateralAccelerationDataRef = NULL;
XPLMDataRef aircraft_airspeedDataRef = NULL;

XPLMDataRef phiDataRef = NULL;
XPLMDataRef thetaDataRef = NULL;
XPLMDataRef psiDataRef = NULL;
XPLMDataRef phiIndicatedDataref = NULL;
XPLMDataRef thetaIndicatedDataref = NULL;
XPLMDataRef psiIndicatedDataref = NULL;
// dataref creation for main rotor angles
XPLMDataRef blade_pitch_mainDataRef = NULL;
XPLMDataRef blade_flapDataRef = NULL;
XPLMDataRef blade_lagDataRef = NULL;
XPLMDataRef blade_pitch_tailDataRef = NULL;
XPLMDataRef rotors_shaft_anglesDataRef = NULL;

// joystick datarefs
XPLMDataRef joystick_cyclic_lateralDataRef = NULL;
XPLMDataRef joystick_cyclic_longitudinalDataRef = NULL;
XPLMDataRef joystick_pedalsDataRef = NULL;
XPLMDataRef joystick_collectiveDataRef = NULL;

// joystick datarefs
XPLMDataRef joystick_trim_cyclic_lateralDataRef = NULL;
XPLMDataRef joystick_trim_cyclic_longitudinalDataRef = NULL;
XPLMDataRef joystick_trim_pedalsDataRef = NULL;
XPLMDataRef joystick_trim_collectiveDataRef = NULL;


// dataref creation for ship position
XPLMDataRef ship_xDataRef = NULL;
XPLMDataRef ship_yDataRef = NULL;
XPLMDataRef ship_zDataRef = NULL;
XPLMDataRef ship_phiDataRef = NULL;
XPLMDataRef ship_theDataRef = NULL;
XPLMDataRef ship_psiDataRef = NULL;

// create ship object loader
const char* F590FolderLocation = "Resources/plugins/XplanecomWidget/F590.obj";
XPLMObjectRef F590ObjectRef;
XPLMObjectLoaded_f F590ObjectLoaded(
																		XPLMObjectRef inObject,
																		void* inRefcon);
XPLMDrawInfo_t F590LocationAndAttitude;
XPLMInstanceRef F590InstanceRef;

const char* F590InstancedDataRefs[6][50]={
			"sim/multiplayer/position/plane1_x",
			"sim/multiplayer/position/plane1_y",
			"sim/multiplayer/position/plane1_z",
			"sim/multiplayer/position/plane1_phi",
			"sim/multiplayer/position/plane1_the",
			"sim/multiplayer/position/plane1_psi"};


void 	F590LoadObjectAsync(
													const char* inPath,
													XPLMObjectLoaded_f inCallback,
													void* inRefcon);

// create a customized flightloop callback
XPLMFlightLoopID MBDynFrameSimFlightLoop;


float initialAnglesValue[5] = {10};
// initialize socket and structure
int socketInfo;
struct sockaddr_in server;
int receivedArray=1;

// socket function declaration that is repeated during the flight loop
static float ReceiveDataFromSocket(
                                 				float                inElapsedSinceLastCall,
                                   			float                inElapsedTimeSinceLastFlightLoop,
                                   			int                  inCounter,
                                   			void *               inRefcon);

// structure definition
struct{
// helo
double latitudeReceived;
double longitudeReceived;
double elevationReceived;
float airspeedReceived;
float vertspeedReceived;
float lateralaccReceived;
float phiReceived;
float thetaReceived;
float psiReceived;
float BLADE_1_pitch;
float BLADE_1_flap;
float BLADE_1_lag;
float BLADE_2_pitch;
float BLADE_2_flap;
float BLADE_2_lag;
float BLADE_3_pitch;
float BLADE_3_flap;
float BLADE_3_lag;
float BLADE_4_pitch;
float BLADE_4_flap;
float BLADE_4_lag;
float BLADE_5_pitch;
float BLADE_5_flap;
float BLADE_5_lag;
// joystick
float JOYSTICK_CYCLIC_LATERAL;
float JOYSTICK_CYCLIC_LONGITUDINAL;
float JOYSTICK_PEDALS;
float JOYSTICK_COLLECTIVE;
float JOYSTICK_TRIM_CYCLIC_LATERAL;
float JOYSTICK_TRIM_CYCLIC_LONGITUDINAL;
float JOYSTICK_TRIM_PEDALS;
float JOYSTICK_TRIM_COLLECTIVE;
float MR_SHAFT_ANGLE;
// ship

double SHIP_X;
double SHIP_Y;
double SHIP_Z;
//float SHIP_X;
//float SHIP_Y;
//float SHIP_Z;
float SHIP_PHI;
float SHIP_THE;
float SHIP_PSI;
}
// positionAndAttitude_ is a structure
positionAndAttitude_;

// values initialization, VERBOSE BUT CLEAR TO UNDERSTAND
double aircraft_latitude;
double aircraft_longitude;
double aircraft_elevation;
double aircraft_localX;
double aircraft_localY;
double aircraft_localZ;
float aircraft_airspeed;
float aircraft_vertspeed;
float aircraft_lateralacc;
float aircraft_phi;
float aircraft_theta;
float aircraft_psi;

double ship_latitude;
double ship_longitude;
double ship_elevation;
double ship_localX;
double ship_localY;
double ship_localZ;
float ship_phi;
float ship_theta;
float ship_psi;

float thetaMain[5];
float betaMain[5];
float epsMain[5];
float thetaTail[5];
float joystickNormalizedPositions[4];
float joystickTrimNormalizedPositions[4];
float rotorsShaftAngles[2];

// array to override internal fdm (20 values can be overridden, interested only in the first)
int deactivateFlag[1];
// array to reactivate internal fdm
int reactivateFlag[1];
// button state for the widget
static XPWidgetID FDMbuttonState[1] = {NULL};


// load ship asynchronously
//void F590LoadObjectAsync(const char* inPath, XPLMObjectLoaded_f inCallback, void* inRefcon){





//socket functions
int CreateSocket(const char* host, int socketport){
	//create socket
  socketInfo = socket(AF_INET,SOCK_DGRAM,0);
  // assign values
  server.sin_addr.s_addr = inet_addr(Host);
  server.sin_family = AF_INET;
  server.sin_port = htons(socketPort);
  // bind to socket
  if(bind(socketInfo,(struct sockaddr *) &server,sizeof(server))<0){
    std::cerr<< "ERROR ON BINDING" <<'\n';
    return 1;
  }
  std::cout << "Listening from IP address: "<< Host <<", port: "<< socketPort <<" ..." << '\n';
		return 0;
}

//socket data reception
float ReceiveDataFromSocket(       float                inElapsedSinceLastCall,
                                   float                inElapsedTimeSinceLastFlightLoop,
                                   int                  inCounter,
                                   void *               inRefcon){

  // receive incoming message
  receivedArray = recv(socketInfo,&positionAndAttitude_, sizeof(positionAndAttitude_), 0);
  if( receivedArray < 0) {
		std::cout << "ERROR ON RECEIVING: bytes received= "<< receivedArray+1 << '\n';
    return 1;
    }

  // transform the message into something useful
  aircraft_latitude 	= positionAndAttitude_.latitudeReceived;
  aircraft_longitude 	= positionAndAttitude_.longitudeReceived;
  aircraft_elevation 	= positionAndAttitude_.elevationReceived;
	aircraft_airspeed 	= positionAndAttitude_.airspeedReceived;
	aircraft_vertspeed  = positionAndAttitude_.vertspeedReceived;
	aircraft_lateralacc = positionAndAttitude_.lateralaccReceived;
  aircraft_phi 	 			= positionAndAttitude_.phiReceived;
  aircraft_theta 			= positionAndAttitude_.thetaReceived;
  aircraft_psi 	 			= positionAndAttitude_.psiReceived;

	thetaMain[0] = positionAndAttitude_.BLADE_1_pitch;
	betaMain[0]  = positionAndAttitude_.BLADE_1_flap;
	epsMain[0] 	= positionAndAttitude_.BLADE_1_lag;
	thetaMain[1] = positionAndAttitude_.BLADE_2_pitch;
	betaMain[1] = positionAndAttitude_.BLADE_2_flap;
	epsMain[1] = positionAndAttitude_.BLADE_2_lag;
	thetaMain[2] = positionAndAttitude_.BLADE_3_pitch;
	betaMain[2] = positionAndAttitude_.BLADE_3_flap;
	epsMain[2] = positionAndAttitude_.BLADE_3_lag;
	thetaMain[3] = positionAndAttitude_.BLADE_4_pitch;
	betaMain[3] = positionAndAttitude_.BLADE_4_flap;
	epsMain[3] = positionAndAttitude_.BLADE_4_lag;
	thetaMain[4] = positionAndAttitude_.BLADE_5_pitch;
	betaMain[4] = positionAndAttitude_.BLADE_5_flap;
	epsMain[4] = positionAndAttitude_.BLADE_5_lag;
	// for now only the collective value of the tail is provided, unnecessary to be more specific
	thetaTail[0] = positionAndAttitude_.JOYSTICK_PEDALS;
	thetaTail[1] = positionAndAttitude_.JOYSTICK_PEDALS;
	thetaTail[2] = positionAndAttitude_.JOYSTICK_PEDALS;
	thetaTail[3] = positionAndAttitude_.JOYSTICK_PEDALS;
	joystickNormalizedPositions[0] = positionAndAttitude_.JOYSTICK_CYCLIC_LATERAL;
	joystickNormalizedPositions[1] = positionAndAttitude_.JOYSTICK_CYCLIC_LONGITUDINAL;
	joystickNormalizedPositions[2] = positionAndAttitude_.JOYSTICK_PEDALS;
	joystickNormalizedPositions[3] = positionAndAttitude_.JOYSTICK_COLLECTIVE;

	joystickTrimNormalizedPositions[0] = positionAndAttitude_.JOYSTICK_TRIM_CYCLIC_LATERAL;
	joystickTrimNormalizedPositions[1] = positionAndAttitude_.JOYSTICK_TRIM_CYCLIC_LONGITUDINAL;
	joystickTrimNormalizedPositions[2] = positionAndAttitude_.JOYSTICK_TRIM_PEDALS;
	joystickTrimNormalizedPositions[3] = positionAndAttitude_.JOYSTICK_TRIM_COLLECTIVE;

	// main and tail rotor RPM
	rotorsShaftAngles[0] = positionAndAttitude_.MR_SHAFT_ANGLE;
	rotorsShaftAngles[1] = positionAndAttitude_.MR_SHAFT_ANGLE;

	ship_localX = positionAndAttitude_.SHIP_X;
	ship_localY = positionAndAttitude_.SHIP_Y;
	ship_localZ = positionAndAttitude_.SHIP_Z;

	ship_phi 	 = positionAndAttitude_.SHIP_PHI*rad2deg;
	ship_theta = positionAndAttitude_.SHIP_THE*rad2deg;
	ship_psi 	 = positionAndAttitude_.SHIP_PSI*rad2deg;

  // print what is received
	//std::cout << "number of cycles=" << XPLMGetCycleNumber() << '\t';
  //std::cout << "bytes received= " << receivedArray << '\n';
	//transform in local coordinates that can be written as datarefs
	XPLMWorldToLocal(aircraft_latitude,aircraft_longitude,aircraft_elevation,
									 &aircraft_localX,&aircraft_localY,&aircraft_localZ);


	// set the useful data
	XPLMSetDatad(aircraft_xLocalDataRef,aircraft_localX);
	XPLMSetDatad(aircraft_yLocalDataRef,aircraft_localY);
	XPLMSetDatad(aircraft_zLocalDataRef,aircraft_localZ);
	XPLMSetDataf(aircraft_indicatedAltitudeDataRef,aircraft_elevation/mt2ft); // in ft
	XPLMSetDataf(aircraft_airspeedDataRef,aircraft_airspeed);
	XPLMSetDataf(aircraft_verticalSpeedDataRef,aircraft_vertspeed*60.0); // in ft/min
	XPLMSetDataf(aircraft_lateralAccelerationDataRef,aircraft_lateralacc); // in m/s^2
	XPLMSetDataf(phiDataRef,aircraft_phi);
	XPLMSetDataf(thetaDataRef,aircraft_theta);
	XPLMSetDataf(psiDataRef,aircraft_psi);
	// for the artificial horizon
	XPLMSetDataf(phiIndicatedDataref,aircraft_phi);
	XPLMSetDataf(thetaIndicatedDataref,aircraft_theta);
	XPLMSetDataf(psiIndicatedDataref,aircraft_psi);


	XPLMSetDatavf(blade_pitch_mainDataRef,thetaMain,0,5);
	XPLMSetDatavf(blade_flapDataRef,betaMain,0,5);
	XPLMSetDatavf(blade_lagDataRef,epsMain,0,5);
	XPLMSetDatavf(blade_pitch_tailDataRef,thetaTail,0,5);
	XPLMSetDataf(joystick_cyclic_lateralDataRef,joystickNormalizedPositions[0]);
	XPLMSetDataf(joystick_cyclic_longitudinalDataRef,joystickNormalizedPositions[1]);
	XPLMSetDataf(joystick_pedalsDataRef,joystickNormalizedPositions[2]);
	XPLMSetDataf(joystick_collectiveDataRef,0.5*(joystickNormalizedPositions[3]+1)); // from 0 to 1

	XPLMSetDataf(joystick_trim_cyclic_lateralDataRef,joystickTrimNormalizedPositions[0]);
	XPLMSetDataf(joystick_trim_cyclic_longitudinalDataRef,joystickTrimNormalizedPositions[1]);
	XPLMSetDataf(joystick_trim_pedalsDataRef,joystickTrimNormalizedPositions[2]);
	XPLMSetDataf(joystick_trim_collectiveDataRef,joystickTrimNormalizedPositions[3]);


	XPLMSetDatavf(rotors_shaft_anglesDataRef,rotorsShaftAngles,0,2);

	XPLMSetDatad(ship_xDataRef,ship_localX);
	XPLMSetDatad(ship_yDataRef,ship_localY);
	XPLMSetDatad(ship_zDataRef,ship_localZ);

	XPLMSetDataf(ship_phiDataRef,ship_phi);
	XPLMSetDataf(ship_theDataRef,ship_theta);
	XPLMSetDataf(ship_psiDataRef,ship_psi);
/*
	std::cout << "ship phi= "  << ship_phi << '\t';
	std::cout << "ship theta= " << ship_theta << '\t';
	std::cout << "ship psi= " << ship_psi << '\n';
	std::cout << "ship x= "  << ship_localX << '\t';
	std::cout << "ship y= " << ship_localY << '\t';
	std::cout << "ship elevation= " << ship_localZ << '\n';
*/

	// create the structure containing the data for the ship
 	XPLMDrawInfo_t F590LocationAndAttitude = {sizeof(XPLMDrawInfo_t),
																						ship_localX,
																						ship_localY,
																						ship_localZ,
																						ship_theta,
																						ship_psi,
																						ship_phi};

/*
	XPLMInstanceSetPosition(F590InstanceRef,
													&F590LocationAndAttitude,
													);
*/
/*
	// set ship position and attitude
	XPLMDrawObjects(F590ObjectRef,
									1,
									&F590LocationAndAttitude,
									0,
									0);
*/

	return dt;

}


PLUGIN_API int XPluginStart(
							char *		outName,
							char *		outSig,
							char *		outDesc)
{

	XPLMMenuID  MenuXPlaneCom;
	int 				MenuItemXPlaneCom;
	strcpy(outName, "Xplanecom");
	strcpy(outSig, 	"MBDynFrameSim.UDPplugin.Xplanecom");
	strcpy(outDesc, "A plugin that disables Xplane's FDM in order to work as visualizer for MBDynFrameSim");

	// create the menu
	// sub menu element (item of MenuXPlaneCom)
	MenuItemXPlaneCom = XPLMAppendMenuItem(XPLMFindPluginsMenu(),"Internal FDM deactivator",NULL,1);
	// root menu element
	MenuXPlaneCom = XPLMCreateMenu("Internal FDM deactivator", XPLMFindPluginsMenu(),MenuItemXPlaneCom,InstructionsMenuHandler,NULL);
	XPLMAppendMenuItem(MenuXPlaneCom,"activation & network settings",(void *) +1, 1);


	/* NOTE: THE FOLLOWING LINES ARE RESPONSIBLE FOR THE DEACTIVATION
		 OF THE INTERNAL FDM */
	// get dataref handles when plugin starts
	deactivateFlag[0] = 1;
	OverrideFDMDataRef = XPLMFindDataRef("sim/operation/override/override_planepath");
	OverrideJoystickDataRef = XPLMFindDataRef("sim/operation/override/override_joystick");
	AvionicsOn =  XPLMFindDataRef("sim/cockpit2/switches/avionics_power_on");
	// find the useful data
	aircraft_xLocalDataRef 	= XPLMFindDataRef("sim/flightmodel/position/local_x");
	aircraft_yLocalDataRef  = XPLMFindDataRef("sim/flightmodel/position/local_y");
	aircraft_zLocalDataRef  = XPLMFindDataRef("sim/flightmodel/position/local_z");

	aircraft_indicatedAltitudeDataRef = XPLMFindDataRef("sim/flightmodel/misc/h_ind");
	aircraft_airspeedDataRef= XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");
	aircraft_verticalSpeedDataRef = XPLMFindDataRef("sim/flightmodel/position/vh_ind_fpm");
	aircraft_lateralAccelerationDataRef = XPLMFindDataRef("sim/flightmodel/position/local_ay");
	// euler angles
	phiDataRef 	= XPLMFindDataRef("sim/flightmodel/position/phi");
	thetaDataRef  = XPLMFindDataRef("sim/flightmodel/position/theta");
	psiDataRef  = XPLMFindDataRef("sim/flightmodel/position/psi");

	phiIndicatedDataref = XPLMFindDataRef("sim/cockpit/gyros/phi_ind_ahars_pilot_deg");
	thetaIndicatedDataref = XPLMFindDataRef("sim/cockpit/gyros/the_ind_ahars_pilot_deg");
	psiIndicatedDataref = XPLMFindDataRef("sim/cockpit/gyros/psi_ind_ahars_pilot_degm");



	// array datarefs available and not used in our case
	blade_pitch_mainDataRef = XPLMFindDataRef("sim/flightmodel2/wing/elevator1_deg");
	blade_flapDataRef = XPLMFindDataRef("sim/flightmodel2/wing/flap1_deg");
	blade_lagDataRef = XPLMFindDataRef("sim/flightmodel2/wing/rudder1_deg");
	blade_pitch_tailDataRef = XPLMFindDataRef("sim/flightmodel2/wing/elevator2_deg");

	joystick_cyclic_lateralDataRef = XPLMFindDataRef("sim/joystick/yoke_roll_ratio");
	joystick_cyclic_longitudinalDataRef = XPLMFindDataRef("sim/joystick/yoke_pitch_ratio");
	joystick_pedalsDataRef = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");
	joystick_collectiveDataRef = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all");

	joystick_trim_cyclic_lateralDataRef = XPLMFindDataRef("sim/cockpit2/controls/aileron_trim");
	joystick_trim_cyclic_longitudinalDataRef = XPLMFindDataRef("sim/cockpit2/controls/elevator_trim");
	joystick_trim_pedalsDataRef = XPLMFindDataRef("sim/cockpit2/controls/rudder_trim");
	joystick_trim_collectiveDataRef = XPLMFindDataRef("sim/cockpit2/controls/rotor_trim");


	rotors_shaft_anglesDataRef = XPLMFindDataRef("sim/flightmodel2/engines/prop_rotation_angle_deg");

	// ship datarefs

	ship_xDataRef = XPLMFindDataRef("sim/multiplayer/position/plane1_x");
	ship_yDataRef = XPLMFindDataRef("sim/multiplayer/position/plane1_y");
	ship_zDataRef = XPLMFindDataRef("sim/multiplayer/position/plane1_z");
	ship_phiDataRef = XPLMFindDataRef("sim/multiplayer/position/plane1_phi");
	ship_theDataRef = XPLMFindDataRef("sim/multiplayer/position/plane1_the");
	ship_psiDataRef = XPLMFindDataRef("sim/multiplayer/position/plane1_psi");

	XPLMSetDatavf(blade_pitch_mainDataRef,initialAnglesValue,0,5);
	XPLMSetDatavf(blade_flapDataRef,initialAnglesValue,0,5);
	XPLMSetDatavf(blade_lagDataRef,initialAnglesValue,0,5);
	XPLMSetDatavf(blade_pitch_tailDataRef,initialAnglesValue,0,4);
	XPLMSetDatavf(rotors_shaft_anglesDataRef,initialAnglesValue,0,2);
	XPLMSetDatai(AvionicsOn,1);

	// initialize ship angles
	/*
	XPLMSetDataf(ship_phiDataRef,0.0);
	XPLMSetDataf(ship_theDataRef,0.0);
	XPLMSetDataf(ship_psiDataRef,0.0);
	*/
	// initialize ship latitude, longitude, elevation
	// TODO: interactively movable
	ship_latitude  = 40.048215;
	ship_longitude = 11.919200;
	ship_elevation = 0.0;
	XPLMWorldToLocal(ship_latitude,ship_longitude,ship_elevation,
									 &ship_localY,&ship_localX,&ship_localZ);


	// load obj file
	F590ObjectRef = XPLMLoadObject(F590FolderLocation);
	if(F590ObjectRef){
		std::cout << "model found in "<< F590FolderLocation << '\n';
	}
	else{
		std::cout << "model not found in "<< F590FolderLocation << '\n';
	}
	std::cout << "MATTEO MATTEO initialization check" << '\n';
	// register an instance of an xplane object
	F590InstanceRef = XPLMCreateInstance(F590ObjectRef,*F590InstancedDataRefs);


	// create the customized flightloop
	XPLMCreateFlightLoop_t	MBDynFrameSimFlightLoop_structure_ptr = {sizeof(XPLMCreateFlightLoop_t),
																																	 xplm_FlightLoop_Phase_AfterFlightModel,
																																	 ReceiveDataFromSocket,
																																	 NULL};
	MBDynFrameSimFlightLoop = XPLMCreateFlightLoop(&MBDynFrameSimFlightLoop_structure_ptr);
	XPLMScheduleFlightLoop(MBDynFrameSimFlightLoop,-1,1);
	/*
	XPLMScheduleFlightLoop schedules a flight loop callback for future execution.
	If inInterval is negative, it is run in a certain number of frames based on the absolute
	value of the input. If the interval is positive, it is a duration in seconds.
 	If inRelativeToNow is true, ties are interpretted relative to the time this routine
 	is called; otherwise they are relative to the last call time or the time
 	the flight loop was registered (if never called).
	*/
	XPLMSetFlightLoopCallbackInterval(ReceiveDataFromSocket,dt,1,NULL);
	/*
		XPLMSetFlightLoopCallbackInterval sets when a callback will be called.
		Do NOT call it from your callback;
	  use the return value of the callback to change your callback interval from inside
		your callback.
		inInterval is formatted the same way as in XPLMRegisterFlightLoopCallback;
		positive for seconds, negative for cycles, and 0 for deactivating the callback.
		If inRelativeToNow is 1, times are from the time of this call;
		otherwise they are from the time the callback was last called
		(or the time it was registered if it has never been called.
	*/

	// create the structure containing the data for the ship
	XPLMDrawInfo_t F590LocationAndAttitude = {sizeof(XPLMDrawInfo_t),
																						ship_localX,
																						ship_localY,
																						ship_localZ,
																						0.0,
																						0.0,
																						0.0};

	// draw ship object where needed
	XPLMDrawObjects(
									F590ObjectRef,
									1,
									&F590LocationAndAttitude,
									0,
									0);

	// register flightloop callback
	XPLMRegisterFlightLoopCallback(ReceiveDataFromSocket,dt,NULL);

	// flag tells us if the widget is displayed
	gMenuItem = 0;
	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
	//we'll be good citizens and clean it up
	if (gMenuItem == 1)
	{
		// destroy when closing
		XPDestroyWidget(ExternalFDMWidget,1);
		// destroy customized flight loop callback
		XPLMDestroyFlightLoop(MBDynFrameSimFlightLoop);

		// when xplane is closed in the proper manner
	}

	/* Unregister the callback */
	XPLMUnregisterFlightLoopCallback(ReceiveDataFromSocket, NULL);
	/* close the socket */
	std::cout << "Closing the socket..."<< '\n';
	close(socketInfo);
	std::cout << "... socket closed!"<< '\n';

	// unload ship
	if(F590ObjectRef){
		std::cout << "Unloading F590 model..."<< '\n';
		XPLMUnloadObject(F590ObjectRef);
		std::cout << "... model unloaded!"<< '\n';
	}

	// destroy ship instance
	XPLMDestroyInstance(F590InstanceRef);

}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void)  { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }

void InstructionsMenuHandler(void * inMenuRef, void * inItemRef)
{
	switch ( (uintptr_t) inItemRef)
	{

		case 1:	if (gMenuItem == 0)
				{
					CreateWidget(96, 256, 256, 128);	//left, top, right, bottom.
					gMenuItem = 1;
				}
				else
				{
					if(!XPIsWidgetVisible(ExternalFDMWidget))
					XPShowWidget(ExternalFDMWidget);
				}
				break;
	}
}

// This will create our widget dialog.
void CreateWidget(int x, int y, int w, int h)
{
int Index;
int x2 = x + w;
int y2 = y - h;

// Create the Main Widget window.
	ExternalFDMWidget = XPCreateWidget(x, y, x2, y2,
					1,										// Visible
					"MBDynFrameSim external FDM activator",	// desc
					1,										// root
					NULL,									// no container
					xpWidgetClass_MainWindow);
					XPSetWidgetProperty(ExternalFDMWidget, xpProperty_MainWindowType, xpMainWindowStyle_Translucent);
					// Add Close Box to the Main Widget.  Other options are available.  See the SDK Documentation.
					XPSetWidgetProperty(ExternalFDMWidget, xpProperty_MainWindowHasCloseBoxes, 1);


					// Print each line of instructions.
						for (Index=0; Index < 50; Index++)
						{
						// possible to add more lines in the future
						if(strcmp(InstructionsText[Index],"") == 0) {break;}

							// Create a text widget
							FDMDeactivatorTextWidget[Index] = XPCreateWidget(x+10, y-(30+(Index*20)) , x2-20, y-(42+(Index*20)),
							1,	// Visible
							InstructionsText[Index],// desc
							0,		// root
							ExternalFDMWidget,
							xpWidgetClass_Caption);

							if(strcmp(InstructionsText[Index],"  FDM internal/external")==0)
							{
								// create button for activation or deactivation of FDM
								FDMbuttonState[0] = XPCreateWidget(x+60, y-(30+(Index*20)) , x2-10, y-(42+(Index*20)),
								1,
								"",
								0,
								ExternalFDMWidget,
								xpWidgetClass_Button);
								// set it to be a checkbox
								XPSetWidgetProperty(FDMbuttonState[0], xpProperty_ButtonType, xpRadioButton);
								XPSetWidgetProperty(FDMbuttonState[0], xpProperty_ButtonBehavior, xpButtonBehaviorCheckBox);
								XPSetWidgetProperty(FDMbuttonState[0], xpProperty_ButtonState, 0);
							}

						}




					// Register our widget handler
					XPAddWidgetCallback(ExternalFDMWidget, InstructionsHandler);

}



// This is our widget handler.
int	InstructionsHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, long  inParam1, long  inParam2)
{




	if (inMessage == xpMessage_CloseButtonPushed)
	{
		if (gMenuItem == 1)
		{
			XPHideWidget(ExternalFDMWidget);
		}
		return 1;
	}

	// handle any checkbox selection
	if (inMessage == xpMsg_ButtonStateChanged)
	{
		if(XPGetWidgetProperty(FDMbuttonState[0],xpProperty_ButtonState,NULL)==1){
		// create the socket to receive from MBDYN
		std::cout << "Socket creation..."<< '\n';
		CreateSocket(Host, socketPort);
		std::cout << "... socket created!"<< '\n';
		// Override of the internal fdm
		std::cout << "FDM is now external"<< '\n';
		XPLMSetDatavi(OverrideFDMDataRef,deactivateFlag,0,1);
		// Joystick override
		std::cout << "Joystick is now external"<< '\n';
		XPLMSetDatai(OverrideJoystickDataRef,1);
		}
		else{
			// close the socket receiving from MBDYN
			std::cout << "Closing the socket..."<< '\n';
			close(socketInfo);
			std::cout << "... socket closed!"<< '\n';
			// stop Override of the internal fdm
			std::cout << "FDM is back to internal"<< '\n';
			XPLMSetDatavi(OverrideFDMDataRef,deactivateFlag,0,0);
			// stop Joystick override
			std::cout << "Joystick is back to internal"<< '\n';
			XPLMSetDatai(OverrideJoystickDataRef,0);
		}
		return 0;
	}
	return 0;
}
