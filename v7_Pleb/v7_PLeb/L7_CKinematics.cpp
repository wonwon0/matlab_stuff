/*
 * L7_CKinematics.cpp
 *
 *  Created on: 2012-11-20
 *      Author: hlamontagne
 */

#include "L7_CKinematics.h"

#ifdef ROBOT_MODEL_JACOV1
	#include "JacoMico4DOF/L7_CSingularityModule.h"
#endif
#ifdef ROBOT_MODEL_MICO
	#include "JacoMico4DOF/L7_CSingularityModule.h"
#endif
#ifdef ROBOT_MODEL_MICO4DOF
	#include "JacoMico4DOF/L7_CSingularityModule.h"
#endif
#ifdef ROBOT_MODEL_JACOV2_4DOF
	#include "JacoMico4DOF/L7_CSingularityModule.h"
#endif


#include "KinovaMath.h"


#ifdef ROBOT_MODEL_JACOV1
	#include "JacoMico4DOF/L7_CControlInfo.h"
#endif
#ifdef ROBOT_MODEL_MICO
	#include "JacoMico4DOF/L7_CControlInfo.h"
#endif
#ifdef ROBOT_MODEL_MICO4DOF
	#include "JacoMico4DOF/L7_CControlInfo.h"
#endif
#ifdef ROBOT_MODEL_JACOV2_4DOF
	#include "JacoMico4DOF/L7_CControlInfo.h"
#endif




#include <std.h>
#include "Keos.h"

// Max number of loops for the recursive algorithm (not used here)
const int MAX_IK_LOOP = 15;

// Tolerance for the inverse kinematics (not used here)
const float TOLERANCE_IK = 0.00015f;

// Debug purpose
float DetGraph = 0;
int FlagDetGraph = 0;

// Constant for the algorithm
// const float ALPHA_IK = 1.0f; TODO ALEX DELETE ?

/**
 * Constructor of the class. It does not call the method Init().
 */
L7_CKinematics::L7_CKinematics()
{
	m_InitDone = false;
}

/**
 * Destructor of the class.
 */
L7_CKinematics::~L7_CKinematics() {

}

/**
 * This is part of the singleton design pattern.
 *
 * \return An instance to a unique immutable instance of the class.
 *
 */
L7_CKinematics* L7_CKinematics::Instance()
{
	static L7_CKinematics Singleton;
	return &Singleton;
}

/**
 * This method is called within the 10 ms second task's process. For now, it manages the initialization process of the class.
 */
void L7_CKinematics::Task10ms()
{
	if(!m_InitDone)
	{
		Init();
	}
}

/**
 * This method tells if the object(singleton) has been initialized.
 *
 * \return Status of the object's initialization.
 */
bool L7_CKinematics::IsInit()
{
	return m_InitDone;
}

/**
 * This method Initializes the object with default values.
 *
 */
void L7_CKinematics::Init()
{// Initialisation
	L6_CLowLevelManagement* layer6 = L6_CLowLevelManagement::Instance();

	m_EndEffectorRotation[0] = 1.0f;
	m_EndEffectorRotation[1] = 0.0f;
	m_EndEffectorRotation[2] = 0.0f;
	m_EndEffectorRotation[3] = 0.0f;
	m_EndEffectorRotation[4] = 1.0f;
	m_EndEffectorRotation[5] = 0.0f;
	m_EndEffectorRotation[6] = 0.0f;
	m_EndEffectorRotation[7] = 0.0f;
	m_EndEffectorRotation[8] = 1.0f;

	m_D[0] = 0.0f;
	m_D[1] = 0.0f;
	m_D[2] = 0.0f;
	m_D[3] = 0.0f;
	m_D[4] = 0.0f;
	m_D[5] = 0.0f;
	m_D[6] = 0.0f;
	m_D[7] = 0.0f;
	m_D[8] = 0.0f;

	for(int i = 0; i < L7_CTypes::EULER_TRANSLATION_SIZE; i++)
	{
		m_OffSetTranslation[i] = 0.0f;
		m_OffSetOrientation[i] = 0.0f;
	}

	m_RobotType = layer6->robotInspector.GetRobotType();

	layer6->robotInspector.GetEndEffTranslationOffset(m_OffSetTranslation);
	SetD();

	m_InitDone = true;
}

/**
 * This method computes the distance needed accelerate/decelerate from StartVelocity to EndVelocity with an acceleration value represented by AccelerationValue.
 *
 * \return The acceleration distance computed.
 */
float L7_CKinematics::CalculAccelerationDistance(float StartVelocity, float EndVelocity, float AccelerationValue)
{// Computes the distance to stop from a given velocity

	float deltaSpeed;
	float decelTime;
	float decelDistance;

	deltaSpeed = EndVelocity - StartVelocity;
	decelTime = deltaSpeed / AccelerationValue;

	decelDistance = (StartVelocity * decelTime)+ AccelerationValue * ((decelTime*decelTime)/2);

	return decelDistance;

}

/**
 * This method performs the inverse kinematics algorithm. (What are the angles to reach a specific 3D position)
 *
 * \param[in] theta An array containing the value of each actuator.
 * \param[in] Pdes An array containing the target translation.
 * \param[in] Q_des An array containning the rotation matrix of the target orientation.
 * \param[in] FrameType Type of the frame(reference's frame).
 * \param[out] theta_IK This array, at the end of the inverse kinematics, will contains the result(6 angles) of the inverse kinematics process.
 * \param[in] dampingAngularVelocity Value between 0 and 1 which is applied on the angular velocity to damp it.
 * \param[in] endEffectorRotation A rotation matrix that will be applied on the end effector's orientation.
 * \param[in] D It is a vector that represents an offset that will be applied on the end effector's translation. it size is 9 and it contains : [L1, L2, L3, L4, L5, L6, offsetX, offsetY, offsetZ] where Lx is the length of a robot's link.
 * \param[in] laterality Represents the laterality of the arm. Right handed or left handed.
 *
 *	\return The jacobian. (Determinant of the jacobian matrix)
 *
 */




double L7_CKinematics::InverseKinematics(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float Pdes[3], float Rdes[3], float theta_IK[NB_ACTUATOR_MAX_HIGH_LEVEL], float dampingAngularVelocity, ArmLaterality laterality, VectorEuler &VelocityVector)
{
		L6_CLowLevelManagement* lowLevelManagement = L6_CLowLevelManagement::Instance();
		L7_CControlInfo* controlInfoManager = L7_CControlInfo::Instance();
		double detJIK = 1.0f;

		// theta is the input and theta_IK the output. First, copy input in output.
		VectCopy(theta_IK, theta, 6);

		// Transfer in Jaco angles
		float theta_Jaco[6];
		Theta_AlgoToJaco(theta, theta_Jaco);

		bool flagErreur = false;

		// Desired position
		float pxdes = Pdes[0];
		float pydes = Pdes[1];
		float pzdes = Pdes[2];
		float rot4des = Rdes[2];


		// Robot parameters
		#ifdef ROBOT_MODEL_JACOV2_4DOF
				float D1  = 0.2755;
				float D2  = 0.410;
				float e2  = -0.0098;
				float D36 = (0.2073+0.160);
		#endif
		#ifdef ROBOT_MODEL_MICO4DOF
				float D1  = 0.2755;
				float D2  = 0.29;
				float e2  = -0.007;
				float D36 = (0.1233+0.160);
		#endif





		float Erreur[4] = {0,0,0,0};

		// Possible solutions branches
		float theta1_t11;
		float theta1_t12;
		float theta2_t11_t31;
		float theta2_t11_t32;
		float theta2_t12_t31;
		float theta2_t12_t32;
		float theta3_t11_t31;
		float theta3_t11_t32;
		float theta3_t12_t31;
		float theta3_t12_t32;
		float theta4;

		// Radius of the desired position
		float r1 = SQRT(pxdes*pxdes+pydes*pydes);

		// If the desired position is near the singularity
		if ( fabsf(r1) < fabsf(e2*1.2) )
		{
			for (int er = 0; er < 4; er++)
			{
				theta1_t11 = 0;
				theta1_t12 = 0;
				Erreur[er] = 1;
			}
		}
		else
		{// else we compute theta1
		    theta1_t11 = (ATAN2(pydes, pxdes) - ATAN2(e2/r1, SQRT(1-(e2/r1)*(e2/r1))));
		    theta1_t12 = (ATAN2(pydes, pxdes) - ATAN2(e2/r1, -SQRT(1-(e2/r1)*(e2/r1))));
		}

		// Intermediate computation
		float F_t11 = pxdes*COS(theta1_t11)+pydes*SIN(theta1_t11);
		float F_t12 = pxdes*COS(theta1_t12)+pydes*SIN(theta1_t12);

		float N_t11 = F_t11*F_t11 + (pzdes-D1)*(pzdes-D1);
		float N_t12 = F_t12*F_t12 + (pzdes-D1)*(pzdes-D1);

		float w_t11 = (D36*D36+D2*D2-N_t11) / (2*D36*D2);
		float w_t12 = (D36*D36+D2*D2-N_t12) / (2*D36*D2);

		// Singularity management
		if ( fabsf(w_t11) > 0.9999 )
		{//If singularity flag error for this solution
		    theta3_t11_t31 = 0;
		    theta3_t11_t32 = 0;
		    Erreur[0] = 1;
		    Erreur[1] = 1;
		}
		else
		{//Compute theta3 solution 1
		    theta3_t11_t31 = ATAN2(w_t11, SQRT(1-w_t11*w_t11));
		    theta3_t11_t32 = ATAN2(w_t11, -SQRT(1-w_t11*w_t11));
		}

		// Singularity management
		if ( fabsf(w_t11) > 0.9999 )
		{//If singularity flag error for this solution
		    theta3_t12_t31 = 0;
		    theta3_t12_t32 = 0;
		    Erreur[2] = 1;
		    Erreur[3] = 1;
		}
		else
		{//Compute theta3 solution 2
		    theta3_t12_t31 = ATAN2(w_t12, SQRT(1-w_t12*w_t12));
		    theta3_t12_t32 = ATAN2(w_t12, -SQRT(1-w_t12*w_t12));
		}

		// Intermediate computation
		float H_t11_t31 = D2-D36*SIN(theta3_t11_t31);
		float H_t11_t32 = D2-D36*SIN(theta3_t11_t32);
		float H_t12_t31 = D2-D36*SIN(theta3_t12_t31);
		float H_t12_t32 = D2-D36*SIN(theta3_t12_t32);

		float G_t11_t31 = -D36*COS(theta3_t11_t31);
		float G_t11_t32 = -D36*COS(theta3_t11_t32);
		float G_t12_t31 = -D36*COS(theta3_t12_t31);
		float G_t12_t32 = -D36*COS(theta3_t12_t32);

		float L_t11_t31 = pxdes*COS(theta1_t11) + pydes*SIN(theta1_t11);
		float L_t11_t32 = pxdes*COS(theta1_t11) + pydes*SIN(theta1_t11);
		float L_t12_t31 = pxdes*COS(theta1_t12) + pydes*SIN(theta1_t12);
		float L_t12_t32 = pxdes*COS(theta1_t12) + pydes*SIN(theta1_t12);

		float M_t11_t31 = pzdes-D1;
		float M_t11_t32 = pzdes-D1;
		float M_t12_t31 = pzdes-D1;
		float M_t12_t32 = pzdes-D1;

		// Compute theta 2
		if ( SQRT(fabsf(H_t11_t31*H_t11_t31+G_t11_t31*G_t11_t31)) < 0.00001 )
		{// If singularity raise flag
		    theta2_t11_t31 = 0;
		    Erreur[0] = 1;
		}
		else
		{//Compute theta 2 solution 1
		    theta2_t11_t31 = ATAN2(M_t11_t31,L_t11_t31)-ATAN2(G_t11_t31,H_t11_t31);
		}


		if ( SQRT(fabsf(H_t11_t32*H_t11_t32+G_t11_t32*G_t11_t32)) < 0.00001 )
		{// If singularity raise flag
		    theta2_t11_t32 = 0;
		    Erreur[1] = 1;

		}
		else
		{//Compute theta 2 solution 2
		    theta2_t11_t32 = ATAN2(M_t11_t32,L_t11_t32)-ATAN2(G_t11_t32,H_t11_t32);
		}

		if ( SQRT(fabsf(H_t12_t31*H_t12_t31+G_t12_t31*G_t12_t31)) < 0.00001 )
		{// If singularity raise flag
		    theta2_t12_t31 = 0;
		    Erreur[2] = 1;
		}
		else
		{//Compute theta 2 solution 3
		    theta2_t12_t31 = ATAN2(M_t12_t31,L_t12_t31)-ATAN2(G_t12_t31,H_t12_t31);
		}

		if ( SQRT(fabsf(H_t12_t32*H_t12_t32+G_t12_t32*G_t12_t32)) < 0.00001 )
		{// If singularity raise flag
		    theta2_t12_t32 = 0;
		    Erreur[3] = 1;
		}
		else
		{//Compute theta 2 solution 4
		    theta2_t12_t32 = ATAN2(M_t12_t32,L_t12_t32)-ATAN2(G_t12_t32,H_t12_t32);
		}

		float Solution[6][4];

		// Get all the possible solutions together in a table to analyze
		Solution[0][0] = theta1_t11;
		Solution[0][1] = theta1_t11;
		Solution[0][2] = theta1_t12;
		Solution[0][3] = theta1_t12;

		Solution[1][0] = theta2_t11_t31;
		Solution[1][1] = theta2_t11_t32;
		Solution[1][2] = theta2_t12_t31;
		Solution[1][3] = theta2_t12_t32;

		Solution[2][0] = theta3_t11_t31;
		Solution[2][1] = theta3_t11_t32;
		Solution[2][2] = theta3_t12_t31;
		Solution[2][3] = theta3_t12_t32;

		Solution[3][0] = rot4des;
		Solution[3][1] = rot4des;
		Solution[3][2] = rot4des;
		Solution[3][3] = rot4des;

		Solution[4][0] = 0;
		Solution[4][1] = 0;
		Solution[4][2] = 0;
		Solution[4][3] = 0;

		Solution[5][0] = 0;
		Solution[5][1] = 0;
		Solution[5][2] = 0;
		Solution[5][3] = 0;

		float SolutionJaco[6][4];

		// Transfer the solution into Jaco angles
		float tempAngles[6];
		float tempAnglesJaco[6];
		for (int h = 0; h < 4; h++)
		{
			for  (int g = 0; g < 6; g++)
			{
				tempAngles[g] = Solution[g][h];
			}

			Theta_AlgoToJaco(tempAngles, tempAnglesJaco);

			for  (int g = 0; g < 6; g++)
			{
				SolutionJaco[g][h] = tempAnglesJaco[g];
				SolutionJaco[3][h] = rot4des*180/PI;//Rotation part directly
			}

		}







	        // Choix solution
	            int NbSolCoudeHaut = 0;
	            int IndexSolution1 = 0;
	            int IndexSolution2 = 0;
	            int IndexSolutionF = 0;
	            float Solution1[6] = {0,0,0,0,0,0};
	            float Solution2[6] = {0,0,0,0,0,0};

				// Reset the angles between 0 and 360
	            for (int h=0; h < 4; h++)
	            {
	            	if (SolutionJaco[1][h] < 0)
	            	{
	            		SolutionJaco[1][h] += 360;
	            	}
	            	if (SolutionJaco[2][h] < 0)
	            	{
	            		SolutionJaco[2][h] += 360;
	            	}

	            }

				// Find the solutions with elbow up
	            for (int h=0; h < 4; h++)
	            {
	                if ( (SolutionJaco[2][h] > 0 && SolutionJaco[2][h] < 180 && laterality == rightHandedness) || (SolutionJaco[2][h] < 360 && SolutionJaco[2][h] > 180 && laterality == leftHandedness) )
	                {
	                    if (NbSolCoudeHaut == 0)
	                    {
	                    	for (int g = 0; g < 6; g++)
	                    	{
								Solution1[g] = SolutionJaco[g][h];
	                    	}
							IndexSolution1 = h;
	                    }
	                    else
	                    {
	                    	for (int g = 0; g < 6; g++)
	                    	{
								Solution2[g] = SolutionJaco[g][h];
	                    	}
	                        IndexSolution2 = h;
	                    }
	                    NbSolCoudeHaut = NbSolCoudeHaut+1;
	                }
	            }

				// Number of elbow up solutions should be 2
	            if (NbSolCoudeHaut != 2)
	            {
	    			flagErreur = true;
	            }

				// Fin the error for joint 1 between different solutions
	            float Theta1_Err1 = Solution1[0] - theta_Jaco[0];
	            float Theta1_Err2 = Solution2[0] - theta_Jaco[0];

	            float Theta1_Err1_360;
	            float Theta1_Err2_360;

	            float SolutionF[6];

				// Find the solution nearest to the actual position and refactor between 0 and 360
	            if (Theta1_Err1 > 0)
	            {
	                Theta1_Err1_360 = Theta1_Err1 - truncf(fabsf(Theta1_Err1)/360)*360;
	            }
	            else
	            {
	                Theta1_Err1_360 = Theta1_Err1 + truncf(fabsf(Theta1_Err1)/360)*360;
	            }
	            if (Theta1_Err1_360 < -180)
	            {
	                Theta1_Err1_360 = Theta1_Err1_360 + 360;
	            }
	            else if (Theta1_Err1_360 > 180)
	            {
	                Theta1_Err1_360 = Theta1_Err1_360 - 360;
	            }


	            if (Theta1_Err2 > 0)
	            {
	                Theta1_Err2_360 = Theta1_Err2 - truncf(fabsf(Theta1_Err2)/360)*360;
	            }
	            else
	            {
	                Theta1_Err2_360 = Theta1_Err2 + truncf(fabsf(Theta1_Err2)/360)*360;
	            }
	            if (Theta1_Err2_360 < -180)
	            {
	                Theta1_Err2_360 = Theta1_Err2_360 + 360;
	            }
	            else if (Theta1_Err2_360 > 180)
	            {
	                Theta1_Err2_360 = Theta1_Err2_360 - 360;
	            }

				// Find the closest solution to the actual position
	            if (fabsf(Theta1_Err1_360) < fabsf(Theta1_Err2_360))
	            {
	                VectCopy(SolutionF, Solution1, 6);
	                IndexSolutionF = IndexSolution1;
	            }
	            else
	            {
	            	VectCopy(SolutionF, Solution2, 6);
	                IndexSolutionF = IndexSolution2;
	            }

				// If error
	            if (Erreur[IndexSolutionF] != 0)
	            {
	    			flagErreur = true;
	            }



	            // Rotation infinie
				// Infinite rotation: Get everything back in the absolute range (not 0-360)
	            float DiffTheta1 = SolutionF[0] - theta_Jaco[0];
	            float DiffTheta1_360 =  DiffTheta1;
	            if (DiffTheta1 > 0)
	            {
	                DiffTheta1_360 = DiffTheta1 - truncf(fabsf(DiffTheta1)/360)*360;
	            }
	            else
	            {
	                DiffTheta1_360 = DiffTheta1 + truncf(fabsf(DiffTheta1)/360)*360;
	            }

	            if (DiffTheta1_360 < -180)
	            {
	                DiffTheta1_360 = DiffTheta1_360 + 360;
	            }
	            else if (DiffTheta1_360 > 180)
	            {
	                DiffTheta1_360 = DiffTheta1_360 - 360;
	            }


	            float DiffTheta4 = SolutionF[3] - theta_Jaco[3];
	            float DiffTheta4_360 =  DiffTheta4;
	            if (DiffTheta4 > 0)
	            {
	                DiffTheta4_360 = DiffTheta4 - truncf(fabsf(DiffTheta4)/360)*360;
	            }
	            else
	            {
	                DiffTheta4_360 = DiffTheta4 + truncf(fabsf(DiffTheta4)/360)*360;
	            }

	            if (DiffTheta4_360 < -180)
	            {
	                DiffTheta4_360 = DiffTheta4_360 + 360;
	            }
	            else if (DiffTheta4_360 > 180)
	            {
	                DiffTheta4_360 = DiffTheta4_360 - 360;
	            }


				// Get the final solution
	            theta_IK[0] = theta_Jaco[0] + DiffTheta1_360;
	            theta_IK[1] = SolutionF[1];
	            theta_IK[2] = SolutionF[2];
	            theta_IK[3] = theta_Jaco[3] + DiffTheta4_360;



		// If DeltaTheta is too big, raise a flag
		float DeltaTheta[4];
		VectSubs(theta_IK, theta_Jaco, DeltaTheta, 4);

		for (int h = 0; h < 4 ; h++)
		{
			if (DeltaTheta[h] > 10)
			{
				flagErreur = true;
			}
		}




		////////////////////////////////////////////////////////////////////////////////////
		// Slow speed when near base							A.Lecours, H. Lamontagne
		////////////////////////////////////////////////////////////////////////////////////
		float rayon = SQRT(pxdes*pxdes + pydes*pydes);

		float r_L1 = 0.040;//Radius 1
		float r_S1 = 0.015;//Radius 2
		float r_H1 = 0.010;//Radius Heigth

		float r_L2 = 0.040 + 0.14;
		float r_S2 = 0.015 + 0.14;
		float r_H2 = 0.010 + 0.14;

		float ZH1 = 0.3;
		float ZH2 = 0.15;
		float ZL1 = -0.3;
		float ZL2 = -0.15;

		float r_L_actu = r_L1;
		float r_S_actu = r_S1;
		float r_H_actu = r_H1;

		// First reduce the speed if near the base proportionnally to the distance
		float ratiorspeed = 1;
		if(rayon < r_L_actu*4)
		{
			float rdist = fabsf(r_L_actu*4 - rayon);



			if (rayon < r_L_actu*2)
			{
				ratiorspeed = 0;
			}
			else
			{
				ratiorspeed = 1 - (rdist / (r_L_actu*4 - r_L_actu*2));
			}

			if (ratiorspeed > 1)
			{
				ratiorspeed = 1;
			}
			if (ratiorspeed < 0.7)
			{
				ratiorspeed = 0.7;
			}

		}




		////////////////////////////////////////////////////////////////////////////////////
		// Actuators velocity limitation							A.Lecours, H. Lamontagne
		////////////////////////////////////////////////////////////////////////////////////

			float ratioSpeedLimitation[6] = {1, 1, 1, 1, 1, 1};
			float tempMaxActuatorVelocity = 0.0f;

			for (int i = 0; i < DESIRED_NB_ACTUATOR_MAX ; i++)
			{
				// Get actuator i maximum velocity
				lowLevelManagement->deviceInspector.actuator.GetMaxSpeed(i, tempMaxActuatorVelocity);

				// security factor
				tempMaxActuatorVelocity = (tempMaxActuatorVelocity) * 0.99f * ratiorspeed;

				// ratio limitation for actuator i
				if (fabsf(DeltaTheta[i]) > (tempMaxActuatorVelocity * CONTROL_TIME_PERIOD * dampingAngularVelocity))
				{
					ratioSpeedLimitation[i] = (tempMaxActuatorVelocity * CONTROL_TIME_PERIOD * dampingAngularVelocity) / fabsf(DeltaTheta[i]);
				}
			}

			// find maximal limitation
			float min_ratioSpeedLimitation = 1;
			min_ratioSpeedLimitation = MinAngle(ratioSpeedLimitation);

			// Apply limitation
			for (int i = 0; i < DESIRED_NB_ACTUATOR_MAX ; i++)
			{
				theta_IK[i] = theta_Jaco[i] + DeltaTheta[i] * min_ratioSpeedLimitation;
			}



		////////////////////////////////////////////////////////////////////////////////////
		// Joint position limitation
		////////////////////////////////////////////////////////////////////////////////////
		// Verify the joint limitations
		float Theta2Min;
		float Theta2Max;
		float Theta3Min;
		float Theta3Max;

		Theta2Min = controlInfoManager->m_Actuator2Min;
		Theta2Max = controlInfoManager->m_Actuator2Max;
		Theta3Min = controlInfoManager->m_Actuator3Min;
		Theta3Max = controlInfoManager->m_Actuator3Max;

        // Limites et securite
		if(laterality == rightHandedness)
		{
			if (theta_IK[1] < Theta2Min || theta_IK[1] > Theta2Max || theta_IK[2] < Theta3Min || theta_IK[2] > 175)
			{
				flagErreur = true;
			}
		}
		else
		{
			if (theta_IK[1] < Theta2Min || theta_IK[1] > Theta2Max || theta_IK[2] > Theta3Max || theta_IK[2] < 185)
			{
				flagErreur = true;
			}
		}



		////////////////////////////////////////////////////////////////////////////////////
		// Base protection cylinder/cone
		////////////////////////////////////////////////////////////////////////////////////
		// Find the radius to protect the base
	    if (pzdes < ZH2 && pzdes > ZL2)
	    {
	        r_L_actu = r_L2;
	        r_S_actu = r_S2;
	        r_H_actu = r_H2;
	    }
	    else if (pzdes < ZH1 && pzdes > ZH2)
	    {
	        r_L_actu = ( 1- (pzdes - ZH2) / (ZH1 - ZH2)) * (r_L2 - r_L1) + r_L1;
	        r_S_actu = r_L_actu - 0.05;
	        r_H_actu = r_L_actu - 0.09;
	    }
	    else if (pzdes > ZL1 && pzdes < ZL2)
	    {
	        r_L_actu = ((pzdes - ZL1) / (ZL2 - ZL1)) * (r_L2 - r_L1) + r_L1;
	        r_S_actu = r_L_actu - 0.05;
	        r_H_actu = r_L_actu - 0.09;
	    }



		// If the hand is too near the base
        if (rayon < r_H_actu)
        {
        	flagErreur = true;
        }


		////////////////////////////////////////////////////////////////////////////////////
		// External sphere
		////////////////////////////////////////////////////////////////////////////////////
		#ifdef ROBOT_MODEL_JACOV2_4DOF
					float Sp1 = 0.72;//Sphere 1 : Slow
					float Sp2 = 0.75;//Sphere 2 : Slide
					float Sp3 = 0.76;//Sphere 3 : Stop
		#endif
		#ifdef ROBOT_MODEL_MICO4DOF
					float Sp1 = 0.52;
					float Sp2 = 0.55;
					float Sp3 = 0.56;
		#endif

	    float Sphere = SQRT(pxdes*pxdes + pydes*pydes + (pzdes - 0.2755)*(pzdes - 0.2755));//Sphere. Must not include base heigth
        if (Sphere > Sp3)//If too far
        {
        	flagErreur = true;
        }



		// Direct kinematics
		float ai[30];
		float ti[30];

		float cinDirectResultFin[128];
		float Rotation4;


		float theta_IKAlgo[6];
		Theta_JacoToAlgo(theta_IK, theta_IKAlgo);
		DirectKinematics(cinDirectResultFin, Rotation4, theta_IKAlgo);
		CalculAi(cinDirectResultFin, theta_IKAlgo, ai);
		CalculTi(cinDirectResultFin, ai, ti);


		// Hard protection zone
		int zoneCount = lowLevelManagement->robotInspector.GetProtectionZoneQuantity();

		float Zone1_Point[13] = {0.0f,0.0f,0.0f,    0.0f,0.0f,0.0f,      0.0f,0.0f,0.0f,     0.0f,0.0f,0.0f, 0.0f};
		float Zone1_Z[3] = {0.0f, 0.0f, 0.0f};

		for(int i = 0; i < (PROTECTION_ZONE_LIST_SIZE + INTERNAL_PROTECTION_ZONE_LIST_SIZE); i++)
		{
	    	if (i < PROTECTION_ZONE_LIST_SIZE && lowLevelManagement->robotInspector.IsZoneActivated(i))
	    	{
				  //On place les points de chacunes des zones dans un tableau afin de les utiliser dans l'algorithme.
	    		lowLevelManagement->robotInspector.GetRawCoord(i,Zone1_Point);

				Zone1_Z[2] = -1 * Zone1_Point[12];
	    	}
	    	else if (i >= PROTECTION_ZONE_LIST_SIZE)
	    	{
				  //On place les points de chacunes des zones dans un tableau afin de les utiliser dans l'algorithme.
	    		lowLevelManagement->robotInspector.GetRawCoordInternal(i-PROTECTION_ZONE_LIST_SIZE,Zone1_Point);
				Zone1_Z[2] = -1 * Zone1_Point[12];
	    	}

	    	if( (lowLevelManagement->robotInspector.IsZoneActivated(i) && i < PROTECTION_ZONE_LIST_SIZE) || i >= PROTECTION_ZONE_LIST_SIZE)
			{
				if((lowLevelManagement->robotInspector.GetProtectionZone(i).GetZoneType() == ZoneType_NoGo && i < PROTECTION_ZONE_LIST_SIZE ) || (lowLevelManagement->robotInspector.GetInternalProtectionZone(i-PROTECTION_ZONE_LIST_SIZE).GetZoneType() == ZoneType_NoGo && i >= PROTECTION_ZONE_LIST_SIZE) )
				{
					if  (ti[9] < Zone1_Point[0]  && ti[9] > Zone1_Point[6] && ti[10] > Zone1_Point[4] && ti[10] < Zone1_Point[10] && ti[11] < Zone1_Point[2] && (ti[11] > (Zone1_Point[2] + Zone1_Z[2]) ) )
					{
						flagErreur = true;
					}
					if (ti[12] < Zone1_Point[0]  && ti[12] > Zone1_Point[6] && ti[13] > Zone1_Point[4] && ti[13] < Zone1_Point[10] && ti[14] < Zone1_Point[2] && (ti[14] > (Zone1_Point[2] + Zone1_Z[2]) ) )
					{
						flagErreur = true;
					}
					if (ti[15] < Zone1_Point[0] && ti[15]  > Zone1_Point[6] && ti[16] > Zone1_Point[4] && ti[16] < Zone1_Point[10] && ti[17] < Zone1_Point[2] && (ti[17] > (Zone1_Point[2] + Zone1_Z[2]) ) )
					{
						flagErreur = true;
					}
				}
			}
		}









		// If there is an error we take the initial angles and set the value DetJIK to zero
		if (flagErreur == true)
		{
			detJIK=0;
			VectCopy(theta_IK, theta_Jaco, 6);
		}



		// Raise a cartesian fault if inverse kinematics problem (Faulty ready)
		if(detJIK <= 0)
		{
			controlInfoManager->m_CartesianFaultState = true;
			controlInfoManager->m_FirstReadyNoMove = true;
		}
		else
		{
			controlInfoManager->m_CartesianFaultState = false;
			//controlInfoManager->m_FirstReadyNoMove = false;
		}




		return detJIK;
}

// This computes the DH frame position
void L7_CKinematics::CalculTi(float tin[128], float ain[30], float ti[30])
{
	  float t10;
	  float t101;
	  float t103;
	  float t104;
	  float t106;
	  float t108;
	  float t11;
	  float t110;
	  float t112;
	  float t113;
	  float t118;
	  float t12;
	  float t123;
	  float t128;
	  float t13;
	  float t133;
	  float t138;
	  float t14;
	  float t143;
	  float t145;
	  float t147;
	  float t149;
	  float t15;
	  float t16;
	  float t17;
	  float t18;
	  float t19;
	  float t20;
	  float t21;
	  float t22;
	  float t23;
	  float t24;
	  float t25;
	  float t26;
	  float t27;
	  float t28;
	  float t30;
	  float t32;
	  float t34;
	  float t35;
	  float t36;
	  float t37;
	  float t39;
	  float t42;
	  float t44;
	  float t45;
	  float t46;
	  float t47;
	  float t49;
	  float t52;
	  float t54;
	  float t55;
	  float t56;
	  float t61;
	  float t62;
	  float t66;
	  float t67;
	  float t7;
	  float t71;
	  float t72;
	  float t76;
	  float t77;
	  float t8;
	  float t81;
	  float t82;
	  float t86;
	  float t87;
	  float t88;
	  float t9;
	  float t90;
	  float t92;
	  float t94;
	  float t95;
	  float t97;
	  float t99;

	    ti[0] = 0.0;
	    ti[1] = 0.0;
	    ti[2] = 0.0;
	    ti[3] = ain[0];
	    ti[4] = ain[1];
	    ti[5] = ain[2];
	    t7 = tin[32];
	    t8 = ain[3];
	    t9 = t7*t8;
	    t10 = tin[33];
	    t11 = ain[4];
	    t12 = t10*t11;
	    t13 = tin[34];
	    t14 = ain[5];
	    t15 = t13*t14;
	    ti[6] = t9+t12+t15+ti[3];
	    t16 = tin[36];
	    t17 = t16*t8;
	    t18 = tin[37];
	    t19 = t18*t11;
	    t20 = tin[38];
	    t21 = t20*t14;
	    ti[7] = t17+t19+t21+ti[4];
	    t22 = tin[40];
	    t23 = t22*t8;
	    t24 = tin[41];
	    t25 = t24*t11;
	    t26 = tin[42];
	    t27 = t26*t14;
	    ti[8] = t23+t25+t27+ti[5];
	    t28 = tin[48];
	    t30 = tin[52];
	    t32 = tin[56];
	    t34 = t10*t30+t13*t32+t7*t28;
	    t35 = ain[6];
	    t36 = t34*t35;
	    t37 = tin[49];
	    t39 = tin[53];
	    t42 = tin[57];
	    t44 = t10*t39+t13*t42+t7*t37;
	    t45 = ain[7];
	    t46 = t44*t45;
	    t47 = tin[50];
	    t49 = tin[54];
	    t52 = tin[58];
	    t54 = t10*t49+t13*t52+t7*t47;
	    t55 = ain[8];
	    t56 = t54*t55;
	    ti[9] = t9+t12+t15+ti[3]+t36+t46+t56;
	    t61 = t16*t28+t18*t30+t20*t32;
	    t62 = t61*t35;
	    t66 = t16*t37+t18*t39+t20*t42;
	    t67 = t66*t45;
	    t71 = t16*t47+t18*t49+t20*t52;
	    t72 = t71*t55;
	    ti[10] = t17+t19+t21+ti[4]+t62+t67+t72;
	    t76 = t22*t28+t24*t30+t26*t32;
	    t77 = t76*t35;
	    t81 = t22*t37+t24*t39+t26*t42;
	    t82 = t81*t45;
	    t86 = t22*t47+t24*t49+t26*t52;
	    t87 = t86*t55;
	    ti[11] = t23+t25+t27+ti[5]+t77+t82+t87;
	    t88 = tin[64];
	    t90 = tin[68];
	    t92 = tin[72];
	    t94 = t34*t88+t44*t90+t54*t92;
	    t95 = ain[18];
	    t97 = tin[65];
	    t99 = tin[69];
	    t101 = tin[73];
	    t103 = t54*t101+t34*t97+t44*t99;
	    t104 = ain[19];
	    t106 = tin[66];
	    t108 = tin[70];
	    t110 = tin[74];
	    t112 = t34*t106+t44*t108+t54*t110;
	    t113 = ain[20];
	    ti[12] = t103*t104+t112*t113+t94*t95+t12+t15+t36+t46+t56+t9+ti[3];
	    t118 = t61*t88+t66*t90+t71*t92;
	    t123 = t71*t101+t61*t97+t66*t99;
	    t128 = t61*t106+t66*t108+t71*t110;
	    ti[13] = t123*t104+t128*t113+t118*t95+t17+t19+t21+t62+t67+t72+ti[4];
	    t133 = t76*t88+t81*t90+t86*t92;
	    t138 = t86*t101+t76*t97+t81*t99;
	    t143 = t76*t106+t81*t108+t86*t110;
	    ti[14] = t138*t104+t143*t113+t133*t95+t23+t25+t27+t77+t82+t87+ti[5];
	    t145 = ain[9];
	    t147 = ain[10];
	    t149 = ain[11];
	    ti[15] = t103*t147+t112*t149+t94*t145+t12+t15+t36+t46+t56+t9+ti[3];
	    ti[16] = t118*t145+t123*t147+t128*t149+t17+t19+t21+t62+t67+t72+ti[4];
	    ti[17] = t133*t145+t138*t147+t143*t149+t23+t25+t27+t77+t82+t87+ti[5];
	    ti[18] = ti[15];
	    ti[19] = ti[16];
	    ti[20] = ti[17];
	    ti[21] = ti[18];
	    ti[22] = ti[19];
	    ti[23] = ti[20];
	    ti[24] = ti[21];
	    ti[25] = ti[22];
	    ti[26] = ti[23];
	    ti[27] = ti[24];
	    ti[28] = ti[25];
	    ti[29] = ti[26];
}


/**
 * This method takes angles from algorithm's domain and transform them into robot's domain.
 *
 * \param[in] theta_Algo An array that contains angles from the algorithm's domain.
 * \param[out] theta_Jaco An array that contains the conversion from the algorithm's domain to the Jaco's domain.
 */
void  L7_CKinematics::Theta_AlgoToJaco(float theta_Algo[NB_ACTUATOR_MAX_HIGH_LEVEL], float theta_Jaco[NB_ACTUATOR_MAX_HIGH_LEVEL])
{

		#ifdef ROBOT_MODEL_JACOV2_4DOF
				Theta_AlgoToJacoJaco(theta_Algo, theta_Jaco);
		#endif

		#ifdef ROBOT_MODEL_MICO4DOF
				Theta_AlgoToJacoMico(theta_Algo, theta_Jaco);
		#endif

}

#ifdef ROBOT_MODEL_JACOV2_4DOF

void L7_CKinematics::Theta_AlgoToJacoJaco(float theta_Algo[NB_ACTUATOR_MAX_HIGH_LEVEL], float theta_Jaco[NB_ACTUATOR_MAX_HIGH_LEVEL])
{
	float theta[NB_ACTUATOR_MAX_HIGH_LEVEL];

	theta[L6_CTypes::ACTUATOR_1] = theta_Algo[L6_CTypes::ACTUATOR_1];
	theta[L6_CTypes::ACTUATOR_2] = theta_Algo[L6_CTypes::ACTUATOR_2];
	theta[L6_CTypes::ACTUATOR_3] = theta_Algo[L6_CTypes::ACTUATOR_3];
	theta[L6_CTypes::ACTUATOR_4] = theta_Algo[L6_CTypes::ACTUATOR_4];
	theta[L6_CTypes::ACTUATOR_5] = theta_Algo[L6_CTypes::ACTUATOR_5];
	theta[L6_CTypes::ACTUATOR_6] = theta_Algo[L6_CTypes::ACTUATOR_6];

	theta[L6_CTypes::ACTUATOR_1] = -1 * theta[L6_CTypes::ACTUATOR_1];

	float theta_off[NB_ACTUATOR_MAX_HIGH_LEVEL] = {0,(-1 * PI) / 2, (PI) / 2, PI, 0 , 0};

	theta_Jaco[L6_CTypes::ACTUATOR_1] = theta[L6_CTypes::ACTUATOR_1] - theta_off[L6_CTypes::ACTUATOR_1];
	theta_Jaco[L6_CTypes::ACTUATOR_2] = theta[L6_CTypes::ACTUATOR_2] - theta_off[L6_CTypes::ACTUATOR_2];
	theta_Jaco[L6_CTypes::ACTUATOR_3] = theta[L6_CTypes::ACTUATOR_3] - theta_off[L6_CTypes::ACTUATOR_3];
	theta_Jaco[L6_CTypes::ACTUATOR_4] = theta[L6_CTypes::ACTUATOR_4] - theta_off[L6_CTypes::ACTUATOR_4];
	theta_Jaco[L6_CTypes::ACTUATOR_5] = theta[L6_CTypes::ACTUATOR_5] - theta_off[L6_CTypes::ACTUATOR_5];
	theta_Jaco[L6_CTypes::ACTUATOR_6] = theta[L6_CTypes::ACTUATOR_6] - theta_off[L6_CTypes::ACTUATOR_6];

	theta_Jaco[L6_CTypes::ACTUATOR_1] = theta_Jaco[L6_CTypes::ACTUATOR_1] * OPT_180_PI;
	theta_Jaco[L6_CTypes::ACTUATOR_2] = theta_Jaco[L6_CTypes::ACTUATOR_2] * OPT_180_PI;
	theta_Jaco[L6_CTypes::ACTUATOR_3] = theta_Jaco[L6_CTypes::ACTUATOR_3] * OPT_180_PI;
	theta_Jaco[L6_CTypes::ACTUATOR_4] = theta_Jaco[L6_CTypes::ACTUATOR_4] * OPT_180_PI;
	theta_Jaco[L6_CTypes::ACTUATOR_5] = theta_Jaco[L6_CTypes::ACTUATOR_5] * OPT_180_PI;
	theta_Jaco[L6_CTypes::ACTUATOR_6] = theta_Jaco[L6_CTypes::ACTUATOR_6] * OPT_180_PI;
}
#endif

#ifdef ROBOT_MODEL_MICO4DOF

void L7_CKinematics::Theta_AlgoToJacoMico(float theta_Algo[NB_ACTUATOR_MAX_HIGH_LEVEL], float theta_Jaco[NB_ACTUATOR_MAX_HIGH_LEVEL])
{
	float theta[NB_ACTUATOR_MAX_HIGH_LEVEL];

	theta[L6_CTypes::ACTUATOR_1] = theta_Algo[L6_CTypes::ACTUATOR_1];
	theta[L6_CTypes::ACTUATOR_2] = theta_Algo[L6_CTypes::ACTUATOR_2];
	theta[L6_CTypes::ACTUATOR_3] = theta_Algo[L6_CTypes::ACTUATOR_3];
	theta[L6_CTypes::ACTUATOR_4] = theta_Algo[L6_CTypes::ACTUATOR_4];
	theta[L6_CTypes::ACTUATOR_5] = theta_Algo[L6_CTypes::ACTUATOR_5];
	theta[L6_CTypes::ACTUATOR_6] = theta_Algo[L6_CTypes::ACTUATOR_6];

	theta[L6_CTypes::ACTUATOR_1] = -1 * theta[L6_CTypes::ACTUATOR_1];

	float theta_off[NB_ACTUATOR_MAX_HIGH_LEVEL] = {0,(-1 * PI) / 2, (PI) / 2, 270*PI_180, 0 , 0};

	theta_Jaco[L6_CTypes::ACTUATOR_1] = theta[L6_CTypes::ACTUATOR_1] - theta_off[L6_CTypes::ACTUATOR_1];
	theta_Jaco[L6_CTypes::ACTUATOR_2] = theta[L6_CTypes::ACTUATOR_2] - theta_off[L6_CTypes::ACTUATOR_2];
	theta_Jaco[L6_CTypes::ACTUATOR_3] = theta[L6_CTypes::ACTUATOR_3] - theta_off[L6_CTypes::ACTUATOR_3];
	theta_Jaco[L6_CTypes::ACTUATOR_4] = theta[L6_CTypes::ACTUATOR_4] - theta_off[L6_CTypes::ACTUATOR_4];
	theta_Jaco[L6_CTypes::ACTUATOR_5] = theta[L6_CTypes::ACTUATOR_5] - theta_off[L6_CTypes::ACTUATOR_5];
	theta_Jaco[L6_CTypes::ACTUATOR_6] = theta[L6_CTypes::ACTUATOR_6] - theta_off[L6_CTypes::ACTUATOR_6];

	theta_Jaco[L6_CTypes::ACTUATOR_1] = theta_Jaco[L6_CTypes::ACTUATOR_1] * 180.0f / PI;
	theta_Jaco[L6_CTypes::ACTUATOR_2] = theta_Jaco[L6_CTypes::ACTUATOR_2] * 180.0f / PI;
	theta_Jaco[L6_CTypes::ACTUATOR_3] = theta_Jaco[L6_CTypes::ACTUATOR_3] * 180.0f / PI;
	theta_Jaco[L6_CTypes::ACTUATOR_4] = theta_Jaco[L6_CTypes::ACTUATOR_4] * 180.0f / PI;
	theta_Jaco[L6_CTypes::ACTUATOR_5] = theta_Jaco[L6_CTypes::ACTUATOR_5] * 180.0f / PI;
	theta_Jaco[L6_CTypes::ACTUATOR_6] = theta_Jaco[L6_CTypes::ACTUATOR_6] * 180.0f / PI;
}
#endif





/**
 * This method takes angles from robot's domain and transform them into the algorithm's domain.
 *
 * \param[in] theta_Algo An array that contains angles from the algorithm's domain.
 * \param[out] theta_Jaco An array that contains the conversion from the algorithm's domain to the Jaco's domain.
 */
void  L7_CKinematics::Theta_JacoToAlgo(float theta_Jaco[NB_ACTUATOR_MAX_HIGH_LEVEL], float theta_Algo[NB_ACTUATOR_MAX_HIGH_LEVEL])
{

	#ifdef ROBOT_MODEL_JACOV2_4DOF
			Theta_JacoJacoToAlgo(theta_Jaco, theta_Algo);
	#endif

	#ifdef ROBOT_MODEL_MICO4DOF
			Theta_JacoMicoToAlgo(theta_Jaco, theta_Algo);
	#endif

}

#ifdef ROBOT_MODEL_JACOV2_4DOF

void L7_CKinematics::Theta_JacoJacoToAlgo(float theta_Jaco[NB_ACTUATOR_MAX_HIGH_LEVEL], float theta_Algo[NB_ACTUATOR_MAX_HIGH_LEVEL])
{
	float theta_off[NB_ACTUATOR_MAX_HIGH_LEVEL] = {0,(-1 * PI) / 2, (PI) / 2, PI, 0 , 0};

	theta_Algo[L6_CTypes::ACTUATOR_1] = (theta_Jaco[L6_CTypes::ACTUATOR_1] * PI_180) + theta_off[L6_CTypes::ACTUATOR_1];
	theta_Algo[L6_CTypes::ACTUATOR_2] = (theta_Jaco[L6_CTypes::ACTUATOR_2] * PI_180) + theta_off[L6_CTypes::ACTUATOR_2];
	theta_Algo[L6_CTypes::ACTUATOR_3] = (theta_Jaco[L6_CTypes::ACTUATOR_3] * PI_180) + theta_off[L6_CTypes::ACTUATOR_3];
	theta_Algo[L6_CTypes::ACTUATOR_4] = (theta_Jaco[L6_CTypes::ACTUATOR_4] * PI_180) + theta_off[L6_CTypes::ACTUATOR_4];
	theta_Algo[L6_CTypes::ACTUATOR_5] = (theta_Jaco[L6_CTypes::ACTUATOR_5] * PI_180) + theta_off[L6_CTypes::ACTUATOR_5];
	theta_Algo[L6_CTypes::ACTUATOR_6] = (theta_Jaco[L6_CTypes::ACTUATOR_6] * PI_180) + theta_off[L6_CTypes::ACTUATOR_6];

	theta_Algo[L6_CTypes::ACTUATOR_1] = -1 * theta_Algo[L6_CTypes::ACTUATOR_1];
}
#endif

#ifdef ROBOT_MODEL_MICO4DOF

void L7_CKinematics::Theta_JacoMicoToAlgo(float theta_Jaco[NB_ACTUATOR_MAX_HIGH_LEVEL], float theta_Algo[NB_ACTUATOR_MAX_HIGH_LEVEL])
{
	float theta_off[NB_ACTUATOR_MAX_HIGH_LEVEL] = {0,(-1 * PI) / 2, (PI) / 2, 270*PI_180, 0 , 0};

	theta_Algo[L6_CTypes::ACTUATOR_1] = (theta_Jaco[L6_CTypes::ACTUATOR_1] * PI_180) + theta_off[L6_CTypes::ACTUATOR_1];
	theta_Algo[L6_CTypes::ACTUATOR_2] = (theta_Jaco[L6_CTypes::ACTUATOR_2] * PI_180) + theta_off[L6_CTypes::ACTUATOR_2];
	theta_Algo[L6_CTypes::ACTUATOR_3] = (theta_Jaco[L6_CTypes::ACTUATOR_3] * PI_180) + theta_off[L6_CTypes::ACTUATOR_3];
	theta_Algo[L6_CTypes::ACTUATOR_4] = (theta_Jaco[L6_CTypes::ACTUATOR_4] * PI_180) + theta_off[L6_CTypes::ACTUATOR_4];
	theta_Algo[L6_CTypes::ACTUATOR_5] = (theta_Jaco[L6_CTypes::ACTUATOR_5] * PI_180) + theta_off[L6_CTypes::ACTUATOR_5];
	theta_Algo[L6_CTypes::ACTUATOR_6] = (theta_Jaco[L6_CTypes::ACTUATOR_6] * PI_180) + theta_off[L6_CTypes::ACTUATOR_6];

	theta_Algo[L6_CTypes::ACTUATOR_1] = -1 * theta_Algo[L6_CTypes::ACTUATOR_1];
}
#endif



/**
 * this computes Ai, an intermediate value of each HD frame. It is needed by the inverse kinematics.
 *
 * \param[in] T The homogeneous transform matrix[4 x 4] of each HD frame.
 * \param[in] theta Angles of each actuator.
 * \param[out] The result. A 3D vector for each HD frame.
 */
void L7_CKinematics::CalculAi(float T[128], float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float aiout[30])
{

		#ifdef ROBOT_MODEL_JACOV2_4DOF
		CalculAiJaco(T, theta, aiout);
		#endif

		#ifdef ROBOT_MODEL_MICO4DOF
		CalculAiMico(T, theta, aiout);
		#endif

}

#ifdef ROBOT_MODEL_MICO4DOF
/**
 * this computes Ai, an intermediate value of each HD frame of Mico. It is needed by the inverse kinematics.
 *
 * \param[in] T The homogeneous transform matrix[4 x 4] of each HD frame.
 * \param[in] theta Angles of each actuator.
 * \param[out] The result. A 3D vector for each HD frame.
 */
void L7_CKinematics::CalculAiMico(float T[128], float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float aiout[30])
{

	 	 float t1;
	  float t2;
	  float t3;

	    aiout[0] = 0.0;
	    aiout[1] = 0.0;
	    aiout[2] = 0.2755;
	    t1 = theta[1];
	    t2 = COS(t1);
	    aiout[3] = 0.29*t2;
	    t3 = SIN(t1);
	    aiout[4] = 0.29*t3;
	    aiout[5] = 0.0;
	    aiout[6] = 0.0;
	    aiout[7] = 0.0;
	    aiout[8] = -0.7E-2;
	    aiout[9] = 0.0;
	    aiout[10] = 0.0;
	    aiout[11] = -0.2833;
	    aiout[12] = 0.0;
	    aiout[13] = 0.0;
	    aiout[14] = 0.0;
	    aiout[15] = 0.0;
	    aiout[16] = 0.0;
	    aiout[17] = 0.0;
	    aiout[18] = 0.0;
	    aiout[19] = 0.0;
	    aiout[20] = -0.1233;
	    aiout[21] = 0.0;
	    aiout[22] = 0.0;
	    aiout[23] = 0.0;
	    aiout[24] = 0.0;
	    aiout[25] = 0.0;
	    aiout[26] = 0.0;
	    aiout[27] = T[3];
	    aiout[28] = T[7];
	    aiout[29] = T[11];

}
#endif

#ifdef ROBOT_MODEL_JACOV2_4DOF
/**
 * this computes Ai, an intermediate value of each HD frame of Jaco. It is needed by the inverse kinematics.
 *
 * \param[in] T The homogeneous transform matrix[4 x 4] of each DH frame.
 * \param[in] theta Angles of each actuator.
 * \param[out] The result. A 3D vector for each DH frame.
 */
void L7_CKinematics::CalculAiJaco(float T[128], float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float aiout[30])
{

	  float t1;
	  float t2;
	  float t3;

	    aiout[0] = 0.0;
	    aiout[1] = 0.0;
	    aiout[2] = 0.2755;
	    t1 = theta[1];
	    t2 = COS(t1);
	    aiout[3] = 0.41*t2;
	    t3 = SIN(t1);
	    aiout[4] = 0.41*t3;
	    aiout[5] = 0.0;
	    aiout[6] = 0.0;
	    aiout[7] = 0.0;
	    aiout[8] = -0.98E-2;
	    aiout[9] = 0.0;
	    aiout[10] = 0.0;
	    aiout[11] = -0.3673;
	    aiout[12] = 0.0;
	    aiout[13] = 0.0;
	    aiout[14] = 0.0;
	    aiout[15] = 0.0;
	    aiout[16] = 0.0;
	    aiout[17] = 0.0;
	    aiout[18] = 0.0;
	    aiout[19] = 0.0;
	    aiout[20] = -0.2073;
	    aiout[21] = 0.0;
	    aiout[22] = 0.0;
	    aiout[23] = 0.0;
	    aiout[24] = 0.0;
	    aiout[25] = 0.0;
	    aiout[26] = 0.0;
	    aiout[27] = T[3];
	    aiout[28] = T[7];
	    aiout[29] = T[11];
}
#endif




void L7_CKinematics::CalculJIKFixe(float D[9], float rb[9], float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float Jq1[72])
{

	#ifdef ROBOT_MODEL_JACOV2_4DOF
			CalculJIKFixeJaco(D, rb, theta, Jq1);
	#endif

	#ifdef ROBOT_MODEL_MICO4DOF
			CalculJIKFixeMico(D, rb, theta, Jq1);
	#endif

}

#ifdef ROBOT_MODEL_MICO4DOF
/**
 * This method computes the jacobian matrix for the inverse kinematix for a fixed frame.
 *
 * \param[in] D It is a vector that represents an offset that will be applied on the end effector's translation. it size is 9 and it contains : [L1, L2, L3, L4, L5, L6, offsetX, offsetY, offsetZ] where Lx is the length of a robot's link.
 * \param[in] rb It is a vector that represents an offset that will be applied on the orientation.
 * \param[in] theta Actual values of each joint
 * \param[out] Jq1 The result. A jacobian matrix.
 */
void L7_CKinematics::CalculJIKFixeMico(float D[9], float rb[9], float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float Jq1[72])
{
	for (int i=0; i<72; i++)
	{
		Jq1[i] = 0;
	}
}
#endif

#ifdef ROBOT_MODEL_JACOV2_4DOF
/**
 * This method computes the jacobian matrix for the inverse kinematix for a fixed frame.
 *
 * \param[in] D It is a vector that represents an offset that will be applied on the end effector's translation. it size is 9 and it contains : [L1, L2, L3, L4, L5, L6, offsetX, offsetY, offsetZ] where Lx is the length of a robot's link.
 * \param[in] rb It is a vector that represents an offset that will be applied on the orientation.
 * \param[in] theta Actual values of each joint
 * \param[out] Jq1 The result. A jacobian matrix.
 */
void L7_CKinematics::CalculJIKFixeJaco(float D[9], float rb[9], float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float Jq1[72])
{

	for (int i=0; i<72; i++)
	{
		Jq1[i] = 0;
	}
}
#endif


/**
 * This method set an offset to the robot's end effector.
 *
 * \param[in] offset The values representing the offset.
 */
void L7_CKinematics::SetOffSetTranslation(float offset[L7_CTypes::EULER_TRANSLATION_SIZE])
{
	for(int i = 0; i < L7_CTypes::EULER_TRANSLATION_SIZE; i++)
	{
		m_OffSetTranslation[i] = offset[i];
	}
}

/**
 * This method compute an abstract direct kinematics. From an array of NB_ACTUATOR_MAX_HIGH_LEVEL angles, it computes a space position/orientation. Depending on the type of the robot, it call
 * the right DirectKinematics method.
 *
 * \param[in] D It is a vector that represents an offset that will be applied on the end effector's translation. it size is 9 and it contains : [L1, L2, L3, L4, L5, L6, offsetX, offsetY, offsetZ] where Lx is the length of a robot's link.
 * \param[in] rb It is a vector that represents an offset that will be applied on the orientation.
 * \param[out] Tout Result of the direct kinematics
 * \param[in] theta Actual angles of each actuator.
 *
 */
void L7_CKinematics::DirectKinematics(float Tout[128], float &Rotation4, float theta[NB_ACTUATOR_MAX_HIGH_LEVEL])
{

	#ifdef ROBOT_MODEL_JACOV2_4DOF
			DirectKinematicsJaco(Tout, Rotation4, theta);
	#endif

	#ifdef ROBOT_MODEL_MICO4DOF
			DirectKinematicsMico(Tout, Rotation4, theta);
	#endif

}


#ifdef ROBOT_MODEL_JACOV2_4DOF
/**
 * This method compute an abstract direct kinematics for the robotic arm Jaco. From an array of NB_ACTUATOR_MAX_HIGH_LEVEL angles, it computes a space position/orientation.
 *
 * \param[in] D It is a vector that represents an offset that will be applied on the end effector's translation. it size is 9 and it contains : [L1, L2, L3, L4, L5, L6, offsetX, offsetY, offsetZ] where Lx is the length of a robot's link.
 * \param[in] rb It is a vector that represents an offset that will be applied on the orientation.
 * \param[out] Tout Result of the direct kinematics
 * \param[in] theta Actual angles of each actuator.
 *
 */
void L7_CKinematics::DirectKinematicsJaco(float Tout[128], float &Rotation4, float theta[NB_ACTUATOR_MAX_HIGH_LEVEL])
{
	  float t1;
	  float t10;
	  float t11;
	  float t13;
	  float t14;
	  float t18;
	  float t19;
	  float t2;
	  float t24;
	  float t26;
	  float t28;
	  float t3;
	  float t33;
	  float t34;
	  float t4;
	  float t41;
	  float t42;
	  float t43;
	  float t46;
	  float t6;
	  float t7;
	  float t8;

		float theta1 = theta[0];
		float theta2 = theta[1];
		float theta3 = theta[2];
		float theta4 = theta[3];

		float thetaJacotemp[6];
		Theta_AlgoToJaco(theta, thetaJacotemp);

		Rotation4 = thetaJacotemp[3] * PI / 180.0f;


	    t1 = COS(theta1);
	    t2 = COS(theta2);
	    t3 = t1*t2;
	    t4 = COS(theta3);
	    t6 = SIN(theta2);
	    t7 = t1*t6;
	    t8 = SIN(theta3);
	    t10 = t3*t4+t7*t8;
	    t11 = COS(theta4);
	    t13 = SIN(theta1);
	    t14 = SIN(theta4);
	    Tout[0] = t10*t11-t13*t14;
	    Tout[1] = t10*t14+t13*t11;
	    t18 = t3*t8;
	    t19 = t7*t4;
	    Tout[2] = -t18+t19;
	    Tout[3] = -0.3673*t18+0.3673*t19+0.98E-2*t13+0.41*t3;
	    t24 = t13*t2;
	    t26 = t13*t6;
	    t28 = t24*t4+t26*t8;
	    Tout[4] = t1*t14+t28*t11;
	    Tout[5] = -t1*t11+t28*t14;
	    t33 = t24*t8;
	    t34 = t26*t4;
	    Tout[6] = -t33+t34;
	    Tout[7] = -0.3673*t33+0.3673*t34-0.98E-2*t1+0.41*t24;
	    t41 = -t2*t8+t6*t4;
	    Tout[8] = t41*t11;
	    Tout[9] = t41*t14;
	    t42 = t6*t8;
	    t43 = t2*t4;
	    Tout[10] = -t42-t43;
	    t46 = 0.41*t6;
	    Tout[11] = 0.2755-0.3673*t42-0.3673*t43+t46;
	    Tout[12] = 0.0;
	    Tout[13] = 0.0;
	    Tout[14] = 0.0;
	    Tout[15] = 1.0;
	    Tout[16] = Tout[0];
	    Tout[17] = Tout[1];
	    Tout[18] = Tout[2];
	    Tout[19] = Tout[3];
	    Tout[20] = Tout[4];
	    Tout[21] = Tout[5];
	    Tout[22] = Tout[6];
	    Tout[23] = Tout[7];
	    Tout[24] = Tout[8];
	    Tout[25] = Tout[9];
	    Tout[26] = Tout[10];
	    Tout[27] = Tout[11];
	    Tout[28] = 0.0;
	    Tout[29] = 0.0;
	    Tout[30] = 0.0;
	    Tout[31] = 1.0;
	    Tout[32] = t1;
	    Tout[33] = 0.0;
	    Tout[34] = t13;
	    Tout[35] = 0.0;
	    Tout[36] = Tout[34];
	    Tout[37] = 0.0;
	    Tout[38] = -Tout[32];
	    Tout[39] = 0.0;
	    Tout[40] = 0.0;
	    Tout[41] = 1.0;
	    Tout[42] = 0.0;
	    Tout[43] = 0.2755;
	    Tout[44] = 0.0;
	    Tout[45] = 0.0;
	    Tout[46] = 0.0;
	    Tout[47] = 1.0;
	    Tout[48] = t2;
	    Tout[49] = t6;
	    Tout[50] = 0.0;
	    Tout[51] = 0.41*Tout[48];
	    Tout[52] = Tout[49];
	    Tout[53] = -Tout[48];
	    Tout[54] = 0.0;
	    Tout[55] = t46;
	    Tout[56] = 0.0;
	    Tout[57] = 0.0;
	    Tout[58] = -1.0;
	    Tout[59] = 0.0;
	    Tout[60] = 0.0;
	    Tout[61] = 0.0;
	    Tout[62] = 0.0;
	    Tout[63] = 1.0;
	    Tout[64] = t4;
	    Tout[65] = 0.0;
	    Tout[66] = t8;
	    Tout[67] = 0.0;
	    Tout[68] = Tout[66];
	    Tout[69] = 0.0;
	    Tout[70] = -Tout[64];
	    Tout[71] = 0.0;
	    Tout[72] = 0.0;
	    Tout[73] = 1.0;
	    Tout[74] = 0.0;
	    Tout[75] = -0.98E-2;
	    Tout[76] = 0.0;
	    Tout[77] = 0.0;
	    Tout[78] = 0.0;
	    Tout[79] = 1.0;
	    Tout[80] = t11;
	    Tout[81] = t14;
	    Tout[82] = 0.0;
	    Tout[83] = 0.0;
	    Tout[84] = Tout[81];
	    Tout[85] = -Tout[80];
	    Tout[86] = 0.0;
	    Tout[87] = 0.0;
	    Tout[88] = 0.0;
	    Tout[89] = 0.0;
	    Tout[90] = -1.0;
	    Tout[91] = -0.3673;
	    Tout[92] = 0.0;
	    Tout[93] = 0.0;
	    Tout[94] = 0.0;
	    Tout[95] = 1.0;
	    Tout[96] = 1.0;
	    Tout[97] = 0.0;
	    Tout[98] = 0.0;
	    Tout[99] = 0.0;
	    Tout[100] = 0.0;
	    Tout[101] = 1.0;
	    Tout[102] = 0.0;
	    Tout[103] = 0.0;
	    Tout[104] = 0.0;
	    Tout[105] = 0.0;
	    Tout[106] = 1.0;
	    Tout[107] = 0.0;
	    Tout[108] = 0.0;
	    Tout[109] = 0.0;
	    Tout[110] = 0.0;
	    Tout[111] = 1.0;
	    Tout[112] = 1.0;
	    Tout[113] = 0.0;
	    Tout[114] = 0.0;
	    Tout[115] = 0.0;
	    Tout[116] = 0.0;
	    Tout[117] = 1.0;
	    Tout[118] = 0.0;
	    Tout[119] = 0.0;
	    Tout[120] = 0.0;
	    Tout[121] = 0.0;
	    Tout[122] = 1.0;
	    Tout[123] = 0.0;
	    Tout[124] = 0.0;
	    Tout[125] = 0.0;
	    Tout[126] = 0.0;
	    Tout[127] = 1.0;

}
#endif

#ifdef ROBOT_MODEL_MICO4DOF
/**
 * This method compute an abstract direct kinematics for the robotic arm Mico. From an array of NB_ACTUATOR_MAX_HIGH_LEVEL angles, it computes a space position/orientation.
 *
 * \param[in] D It is a vector that represents an offset that will be applied on the end effector's translation. it size is 9 and it contains : [L1, L2, L3, L4, L5, L6, offsetX, offsetY, offsetZ] where Lx is the length of a robot's link.
 * \param[in] rb It is a vector that represents an offset that will be applied on the orientation.
 * \param[out] Tout Result of the direct kinematics
 * \param[in] theta Actual angles of each actuator.
 *
 */
void L7_CKinematics::DirectKinematicsMico(float Tout[128], float &Rotation4, float theta[NB_ACTUATOR_MAX_HIGH_LEVEL])
{

			float theta1 = theta[0];
			float theta2 = theta[1];
			float theta3 = theta[2];
			float theta4 = theta[3];
			float theta5 = theta[4];
			float theta6 = theta[5];

			float thetaJacotemp[6];
			Theta_AlgoToJaco(theta, thetaJacotemp);

			Rotation4 = thetaJacotemp[3] * PI / 180.0f;

	//Rotation4 = theta4;

	  float t1;
	  float t10;
	  float t11;
	  float t13;
	  float t14;
	  float t18;
	  float t19;
	  float t2;
	  float t24;
	  float t26;
	  float t28;
	  float t3;
	  float t33;
	  float t34;
	  float t4;
	  float t41;
	  float t42;
	  float t43;
	  float t46;
	  float t6;
	  float t7;
	  float t8;

	    t1 = COS(theta1);
	    t2 = COS(theta2);
	    t3 = t1*t2;
	    t4 = COS(theta3);
	    t6 = SIN(theta2);
	    t7 = t1*t6;
	    t8 = SIN(theta3);
	    t10 = t3*t4+t7*t8;
	    t11 = COS(theta4);
	    t13 = SIN(theta1);
	    t14 = SIN(theta4);
	    Tout[0] = t10*t11-t13*t14;
	    Tout[1] = t10*t14+t13*t11;
	    t18 = t3*t8;
	    t19 = t7*t4;
	    Tout[2] = -t18+t19;
	    Tout[3] = -0.2833*t18+0.2833*t19+0.7E-2*t13+0.29*t3;
	    t24 = t13*t2;
	    t26 = t13*t6;
	    t28 = t24*t4+t26*t8;
	    Tout[4] = t1*t14+t28*t11;
	    Tout[5] = -t1*t11+t28*t14;
	    t33 = t24*t8;
	    t34 = t26*t4;
	    Tout[6] = -t33+t34;
	    Tout[7] = -0.2833*t33+0.2833*t34-0.7E-2*t1+0.29*t24;
	    t41 = -t2*t8+t6*t4;
	    Tout[8] = t41*t11;
	    Tout[9] = t41*t14;
	    t42 = t6*t8;
	    t43 = t2*t4;
	    Tout[10] = -t42-t43;
	    t46 = 0.29*t6;
	    Tout[11] = 0.2755-0.2833*t42-0.2833*t43+t46;
	    Tout[12] = 0.0;
	    Tout[13] = 0.0;
	    Tout[14] = 0.0;
	    Tout[15] = 1.0;
	    Tout[16] = Tout[0];
	    Tout[17] = Tout[1];
	    Tout[18] = Tout[2];
	    Tout[19] = Tout[3];
	    Tout[20] = Tout[4];
	    Tout[21] = Tout[5];
	    Tout[22] = Tout[6];
	    Tout[23] = Tout[7];
	    Tout[24] = Tout[8];
	    Tout[25] = Tout[9];
	    Tout[26] = Tout[10];
	    Tout[27] = Tout[11];
	    Tout[28] = 0.0;
	    Tout[29] = 0.0;
	    Tout[30] = 0.0;
	    Tout[31] = 1.0;
	    Tout[32] = t1;
	    Tout[33] = 0.0;
	    Tout[34] = t13;
	    Tout[35] = 0.0;
	    Tout[36] = Tout[34];
	    Tout[37] = 0.0;
	    Tout[38] = -Tout[32];
	    Tout[39] = 0.0;
	    Tout[40] = 0.0;
	    Tout[41] = 1.0;
	    Tout[42] = 0.0;
	    Tout[43] = 0.2755;
	    Tout[44] = 0.0;
	    Tout[45] = 0.0;
	    Tout[46] = 0.0;
	    Tout[47] = 1.0;
	    Tout[48] = t2;
	    Tout[49] = t6;
	    Tout[50] = 0.0;
	    Tout[51] = 0.29*Tout[48];
	    Tout[52] = Tout[49];
	    Tout[53] = -Tout[48];
	    Tout[54] = 0.0;
	    Tout[55] = t46;
	    Tout[56] = 0.0;
	    Tout[57] = 0.0;
	    Tout[58] = -1.0;
	    Tout[59] = 0.0;
	    Tout[60] = 0.0;
	    Tout[61] = 0.0;
	    Tout[62] = 0.0;
	    Tout[63] = 1.0;
	    Tout[64] = t4;
	    Tout[65] = 0.0;
	    Tout[66] = t8;
	    Tout[67] = 0.0;
	    Tout[68] = Tout[66];
	    Tout[69] = 0.0;
	    Tout[70] = -Tout[64];
	    Tout[71] = 0.0;
	    Tout[72] = 0.0;
	    Tout[73] = 1.0;
	    Tout[74] = 0.0;
	    Tout[75] = -0.7E-2;
	    Tout[76] = 0.0;
	    Tout[77] = 0.0;
	    Tout[78] = 0.0;
	    Tout[79] = 1.0;
	    Tout[80] = t11;
	    Tout[81] = t14;
	    Tout[82] = 0.0;
	    Tout[83] = 0.0;
	    Tout[84] = Tout[81];
	    Tout[85] = -Tout[80];
	    Tout[86] = 0.0;
	    Tout[87] = 0.0;
	    Tout[88] = 0.0;
	    Tout[89] = 0.0;
	    Tout[90] = -1.0;
	    Tout[91] = -0.2833;
	    Tout[92] = 0.0;
	    Tout[93] = 0.0;
	    Tout[94] = 0.0;
	    Tout[95] = 1.0;
	    Tout[96] = 1.0;
	    Tout[97] = 0.0;
	    Tout[98] = 0.0;
	    Tout[99] = 0.0;
	    Tout[100] = 0.0;
	    Tout[101] = 1.0;
	    Tout[102] = 0.0;
	    Tout[103] = 0.0;
	    Tout[104] = 0.0;
	    Tout[105] = 0.0;
	    Tout[106] = 1.0;
	    Tout[107] = 0.0;
	    Tout[108] = 0.0;
	    Tout[109] = 0.0;
	    Tout[110] = 0.0;
	    Tout[111] = 1.0;
	    Tout[112] = 1.0;
	    Tout[113] = 0.0;
	    Tout[114] = 0.0;
	    Tout[115] = 0.0;
	    Tout[116] = 0.0;
	    Tout[117] = 1.0;
	    Tout[118] = 0.0;
	    Tout[119] = 0.0;
	    Tout[120] = 0.0;
	    Tout[121] = 0.0;
	    Tout[122] = 1.0;
	    Tout[123] = 0.0;
	    Tout[124] = 0.0;
	    Tout[125] = 0.0;
	    Tout[126] = 0.0;
	    Tout[127] = 1.0;
}
#endif

/**
 * This method returns an array of L7_CTypes::EULER_TRANSLATION_SIZE that represents an offset applied to the endeffector's translation part.
 *
 * \param[out] offset The array containing the offset values.
 *
 */
void L7_CKinematics::GetOffSetTranslation(float offset[L7_CTypes::EULER_TRANSLATION_SIZE])
{
	for(int i = 0; i < L7_CTypes::EULER_TRANSLATION_SIZE; i++)
	{
		offset[i] = m_OffSetTranslation[i];
	}
}

/**
 * This method set an array of L7_CTypes::EULER_ORIENTATION_SIZE that represents an offset applied to the endeffector's orientation part.
 *
 * \param[in] offset The array containing the offset values.
 *
 */
void L7_CKinematics::SetOffSetOrientation(float offset[L7_CTypes::EULER_ORIENTATION_SIZE])
{
	for(int i = 0; i < 3; i++)
	{
		m_OffSetOrientation[i] = offset[i];
	}
}

/**
 * This method returns an array of L7_CTypes::EULER_ORIENTATION_SIZE that represents an offset applied to the endeffector's orientation part.
 *
 * \param[out] offset The array containing the offset values.
 *
 */
void L7_CKinematics::GetOffSetOrientation(float offset[3])
{
	for(int i = 0; i < 3; i++)
	{
		offset[i] = m_OffSetOrientation[i];
	}
}

/**
 * This method takes m_OffSetOrientation and store it in m_EndEffectorRotation as a rotation matrix
 */
void L7_CKinematics::SetEndEffectorRotation()
{
	L7_CSingularityModule* singularityModule = L7_CSingularityModule::Instance();
	singularityModule->EulerToMatRotation(m_OffSetOrientation, m_EndEffectorRotation);
}

/**
 * This method returns an array that represents a rotation matrix applied to the endeffector's orientation part.
 *
 * \param[out] offset The array containing the offset values.
 *
 */
void L7_CKinematics::GetEndEffectorRotation(float endEffectorRotation[9])
{
	L6_CLowLevelManagement* layer6 = L6_CLowLevelManagement::Instance();

	if(layer6->controllerMappingModule.GetDrinkingModeSatus())
	{
		for(int i = 0; i < 9; i++)
		{
			endEffectorRotation[i] = m_EndEffectorRotation[i];
		}
	}
	else
	{
		endEffectorRotation[0] = 1.0f;
		endEffectorRotation[1] = 0.0f;
		endEffectorRotation[2] = 0.0f;
		endEffectorRotation[3] = 0.0f;
		endEffectorRotation[4] = 1.0f;
		endEffectorRotation[5] = 0.0f;
		endEffectorRotation[6] = 0.0f;
		endEffectorRotation[7] = 0.0f;
		endEffectorRotation[8] = 1.0f;
	}

}

/**
 * This method takes the DH parameter and the m_OffSetTranslation array and set them as an offset to the end effector.
 */
void L7_CKinematics::SetD()
{
	L6_CLowLevelManagement* layer6 = L6_CLowLevelManagement::Instance();

	float DhParameters_returned[DH_DIM_1][DH_DIM_2];

	layer6->robotInspector.GetDhParameters(DhParameters_returned);

	m_D[0] = DhParameters_returned[2][0];
	m_D[1] = DhParameters_returned[2][1];
	m_D[2] = DhParameters_returned[2][2];
	m_D[3] = DhParameters_returned[2][3];
	m_D[4] = DhParameters_returned[2][4];
	m_D[5] = DhParameters_returned[2][5];
	m_D[6] = m_OffSetTranslation[0];
	m_D[7] = m_OffSetTranslation[1];
	m_D[8] = m_OffSetTranslation[2];
}

/**
 * this method returns the vector representing the translation offset applied to the end effector.
 *
 * \param[out] An array containing the translation offset vector.
 *
 */
void L7_CKinematics::GetD(float D[9])
{
	L6_CLowLevelManagement* layer6 = L6_CLowLevelManagement::Instance();

	if(layer6->controllerMappingModule.GetDrinkingModeSatus())
	{
		for(int i = 0; i < 9; i++)
		{
			D[i] = m_D[i];
		}
	}
	else
	{
		for(int i = 0; i < 6; i++)
		{
			D[i] = m_D[i];
		}
		for(int i = 6; i < 9; i++)
		{
			D[i] = 0.0f;
		}
	}
}

/**
 * This method computes the theoretical torque, with gravity of each actuator.
 *
 * \param[in] theta Actual angles of each actuator.
 * \param[out] A The result containing torque of each actuator.
 */
void L7_CKinematics::GravityCurrent(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float A[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Mass[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_X[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Y[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Z[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Load[4],float Param_Grav[3] )
{

		#ifdef ROBOT_MODEL_JACOV2_4DOF
			GravityCurrentJaco(theta, A, Param_Mass, Param_X,  Param_Y,  Param_Z,  Param_Load, Param_Grav );
		#endif

		#ifdef ROBOT_MODEL_MICO4DOF
			GravityCurrentMico2(theta, A, Param_Mass, Param_X,  Param_Y,  Param_Z,  Param_Load, Param_Grav );
		#endif

}

#ifdef ROBOT_MODEL_MICO4DOF
/**
 * This method computes the theoretical torque, with gravity of each actuator of the Mico.
 *
 * \param[in] theta Actual angles of each actuator.
 * \param[out] A The result containing torque of each actuator.
 */
void L7_CKinematics::GravityCurrentMico2(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float A[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Mass[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_X[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Y[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Z[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Load[4],float Param_Grav[3] )
{

	float theta1 = theta[0];
	float theta2 = theta[1];
	float theta3 = theta[2];
	float theta4 = theta[3];

	  float t105;
	  float t107;
	  float t110;
	  float t115;
	  float t122;
	  float t125;
	  float t127;
	  float t134;
	  float t145;
	  float t146;
	  float t147;
	  float t148;
	  float t149;
	  float t162;
	  float t165;
	  float t169;
	  float t172;
	  float t173;
	  float t174;
	  float t188;
	  float t19;
	  float t2;
	  float t20;
	  float t204;
	  float t206;
	  float t21;
	  float t211;
	  float t212;
	  float t216;
	  float t217;
	  float t22;
	  float t221;
	  float t222;
	  float t228;
	  float t23;
	  float t24;
	  float t25;
	  float t26;
	  float t28;
	  float t3;
	  float t30;
	  float t31;
	  float t33;
	  float t36;
	  float t4;
	  float t45;
	  float t46;
	  float t47;
	  float t48;
	  float t49;
	  float t5;
	  float t50;
	  float t51;
	  float t53;
	  float t54;
	  float t55;
	  float t56;
	  float t59;
	  float t66;
	  float t67;
	  float t68;
	  float t7;
	  float t70;
	  float t71;
	  float t72;
	  float t75;
	  float t77;
	  float t78;
	  float t8;
	  float t81;
	  float t82;
	  float t85;
	  float t86;
	  float t87;
	  float t88;
	  float t89;
	  float t9;
	  float t91;
	  float t92;
	  float t95;
	  float t98;

	    t2 = Param_Mass[0];
	    t3 = Param_Grav[0];
	    t4 = t2*t3;
	    t5 = SIN(theta1);
	    t7 = Param_Grav[1];
	    t8 = t2*t7;
	    t9 = COS(theta1);
	    t19 = Param_Mass[1];
	    t20 = t19*t3;
	    t21 = COS(theta2);
	    t22 = t5*t21;
	    t23 = t20*t22;
	    t24 = t19*t7;
	    t25 = t9*t21;
	    t26 = t24*t25;
	    t28 = Param_X[1];
	    t30 = SIN(theta2);
	    t31 = t5*t30;
	    t33 = t9*t30;
	    t36 = Param_Y[1];
	    t45 = Param_Mass[2];
	    t46 = t45*t3;
	    t47 = COS(theta3);
	    t48 = t22*t47;
	    t49 = SIN(theta3);
	    t50 = t31*t49;
	    t51 = -t48-t50;
	    t53 = t45*t7;
	    t54 = t25*t47;
	    t55 = t33*t49;
	    t56 = t54+t55;
	    t59 = Param_X[2];
	    t66 = t22*t49;
	    t67 = t31*t47;
	    t68 = -t66+t67;
	    t70 = t25*t49;
	    t71 = t33*t47;
	    t72 = t70-t71;
	    t75 = Param_Z[2];
	    t77 = 0.7E-2*t9;
	    t78 = 0.29*t22;
	    t81 = 0.7E-2*t5;
	    t82 = 0.29*t25;
	    t85 = Param_Mass[3];
	    t86 = Param_Load[0];
	    t87 = t85+t86;
	    t88 = t87*t3;
	    t89 = COS(theta4);
	    t91 = SIN(theta4);
	    t92 = t9*t91;
	    t95 = t87*t7;
	    t98 = -t5*t91+t56*t89;
	    t105 = t85*Param_X[3]+t86*Param_Load[1];
	    t107 = 1/t87;
	    t110 = t9*t89;
	    t115 = t5*t89+t56*t91;
	    t122 = t85*Param_Y[3]+t86*Param_Load[2];
	    t125 = -t68;
	    t127 = -t72;
	    t134 = t85*Param_Z[3]+t86*Param_Load[3];
	    A[0] = -(-t4*t5+t8*t9)*Param_X[0]-(t4*t9+t8*t5)*Param_Z[0]-(-t23+t26)*t28-(-t20*t31+t24*t33)*t36-(-t20*t9-t24*t5)*Param_Z[1]+0.29*t23-0.29*t26-(t46*t51+t53*t56)*t59-(-t46*t9-t53*t5)*Param_Y[2]-(t46*t68+t53*t72)*t75-t46*(t77-t78)-t53*(t81+t82)-(t88*(t51*t89-t92)+t95*t98)*t105*t107-(t88*(t51*t91+t110)+t95*t115)*t122*t107-(t88*t125+t95*t127)*t134*t107-t88*(0.2833*t66-0.2833*t67+t77-t78)-t95*(-0.2833*t70+0.2833*t71+t81+t82);
	    t145 = t20*t33;
	    t146 = t24*t31;
	    t147 = Param_Grav[2];
	    t148 = t19*t147;
	    t149 = t148*t21;
	    t162 = t45*t147;
	    t165 = t21*t47+t30*t49;
	    t169 = -t56;
	    t172 = t21*t49;
	    t173 = t30*t47;
	    t174 = t172-t173;
	    t188 = t87*t147;
	    t204 = -t51;
	    t206 = -t174;
	    t211 = 0.2833*t55;
	    t212 = 0.2833*t54;
	    t216 = 0.2833*t50;
	    t217 = 0.2833*t48;
	    t221 = 0.2833*t172;
	    t222 = 0.2833*t173;
	    A[1] = -(-t145-t146+t149)*t28-(t148*t30+t20*t25+t24*t22)*t36+0.29*t145+0.29*t146-0.29*t149-(t53*t125+t162*t165+t46*t72)*t59-(t162*t174+t46*t169+t53*t51)*t75+0.29*t46*t33+0.29*t53*t31-0.29*t162*t21-(t95*t125*t89+t188*t165*t89+t88*t72*t89)*t105*t107-(t95*t125*t91+t188*t165*t91+t88*t72*t91)*t122*t107-(t188*t206+t95*t204+t88*t56)*t134*t107-t88*(t211+t212-0.29*t33)-t95*(t216+t217-0.29*t31)-t188*(-t221+t222+0.29*t21);
	    t228 = -t165;
	    A[2] = -(t46*t127+t162*t228+t53*t68)*t59-(t162*t206+t53*t204+t46*t56)*t75-(t88*t127*t89+t188*t228*t89+t95*t68*t89)*t105*t107-(t88*t127*t91+t188*t228*t91+t95*t68*t91)*t122*t107-(t88*t169+t188*t174+t95*t51)*t134*t107-t88*(-t212-t211)-t95*(-t217-t216)-t188*(-t222+t221);
	    A[3] = -(-t88*t115+t95*(-t204*t91+t110)-t188*t206*t91)*t105*t107-(t88*t98+t95*(t204*t89+t92)+t188*t206*t89)*t122*t107;


}
#endif

#ifdef ROBOT_MODEL_JACOV2_4DOF
/**
 * This method computes the theoretical torque, with gravity of each actuator of Jaco.
 *
 * \param[in] theta Actual angles of each actuator.
 * \param[out] A The result containing torque of each actuator.
 */
void L7_CKinematics::GravityCurrentJaco(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float A[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Mass[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_X[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Y[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Z[NB_ACTUATOR_MAX_HIGH_LEVEL], float Param_Load[4],float Param_Grav[3] )
{
	  float t105;
	  float t107;
	  float t110;
	  float t115;
	  float t122;
	  float t125;
	  float t127;
	  float t134;
	  float t145;
	  float t146;
	  float t147;
	  float t148;
	  float t149;
	  float t162;
	  float t165;
	  float t169;
	  float t172;
	  float t173;
	  float t174;
	  float t188;
	  float t19;
	  float t2;
	  float t20;
	  float t204;
	  float t206;
	  float t21;
	  float t211;
	  float t212;
	  float t216;
	  float t217;
	  float t22;
	  float t221;
	  float t222;
	  float t228;
	  float t23;
	  float t24;
	  float t25;
	  float t26;
	  float t28;
	  float t3;
	  float t30;
	  float t31;
	  float t33;
	  float t36;
	  float t4;
	  float t45;
	  float t46;
	  float t47;
	  float t48;
	  float t49;
	  float t5;
	  float t50;
	  float t51;
	  float t53;
	  float t54;
	  float t55;
	  float t56;
	  float t59;
	  float t66;
	  float t67;
	  float t68;
	  float t7;
	  float t70;
	  float t71;
	  float t72;
	  float t75;
	  float t77;
	  float t78;
	  float t8;
	  float t81;
	  float t82;
	  float t85;
	  float t86;
	  float t87;
	  float t88;
	  float t89;
	  float t9;
	  float t91;
	  float t92;
	  float t95;
	  float t98;

		float theta1 = theta[0];
		float theta2 = theta[1];
		float theta3 = theta[2];
		float theta4 = theta[3];

	    t2 = Param_Mass[0];
	    t3 = Param_Grav[0];
	    t4 = t2*t3;
	    t5 = SIN(theta1);
	    t7 = Param_Grav[1];
	    t8 = t2*t7;
	    t9 = COS(theta1);
	    t19 = Param_Mass[1];
	    t20 = t19*t3;
	    t21 = COS(theta2);
	    t22 = t5*t21;
	    t23 = t20*t22;
	    t24 = t19*t7;
	    t25 = t9*t21;
	    t26 = t24*t25;
	    t28 = Param_X[1];
	    t30 = SIN(theta2);
	    t31 = t5*t30;
	    t33 = t9*t30;
	    t36 = Param_Y[1];
	    t45 = Param_Mass[2];
	    t46 = t45*t3;
	    t47 = COS(theta3);
	    t48 = t22*t47;
	    t49 = SIN(theta3);
	    t50 = t31*t49;
	    t51 = -t48-t50;
	    t53 = t45*t7;
	    t54 = t25*t47;
	    t55 = t33*t49;
	    t56 = t54+t55;
	    t59 = Param_X[2];
	    t66 = t22*t49;
	    t67 = t31*t47;
	    t68 = -t66+t67;
	    t70 = t25*t49;
	    t71 = t33*t47;
	    t72 = t70-t71;
	    t75 = Param_Z[2];
	    t77 = 0.98E-2*t9;
	    t78 = 0.41*t22;
	    t81 = 0.98E-2*t5;
	    t82 = 0.41*t25;
	    t85 = Param_Mass[3];
	    t86 = Param_Load[0];
	    t87 = t85+t86;
	    t88 = t87*t3;
	    t89 = COS(theta4);
	    t91 = SIN(theta4);
	    t92 = t9*t91;
	    t95 = t87*t7;
	    t98 = -t5*t91+t56*t89;
	    t105 = t85*Param_X[3]+t86*Param_Load[1];
	    t107 = 1/t87;
	    t110 = t9*t89;
	    t115 = t5*t89+t56*t91;
	    t122 = t85*Param_Y[3]+t86*Param_Load[2];
	    t125 = -t68;
	    t127 = -t72;
	    t134 = t85*Param_Z[3]+t86*Param_Load[3];
	    A[0] = -(-t4*t5+t8*t9)*Param_X[0]-(t4*t9+t8*t5)*Param_Z[0]-(-t23+t26)*t28-(-t20*t31+t24*t33)*t36-(-t20*t9-t24*t5)*Param_Z[1]+0.41*t23-0.41*t26-(t46*t51+t53*t56)*t59-(-t46*t9-t53*t5)*Param_Y[2]-(t46*t68+t53*t72)*t75-t46*(t77-t78)-t53*(t81+t82)-(t88*(t51*t89-t92)+t95*t98)*t105*t107-(t88*(t51*t91+t110)+t95*t115)*t122*t107-(t88*t125+t95*t127)*t134*t107-t88*(0.3673*t66-0.3673*t67+t77-t78)-t95*(-0.3673*t70+0.3673*t71+t81+t82);
	    t145 = t20*t33;
	    t146 = t24*t31;
	    t147 = Param_Grav[2];
	    t148 = t19*t147;
	    t149 = t148*t21;
	    t162 = t45*t147;
	    t165 = t21*t47+t30*t49;
	    t169 = -t56;
	    t172 = t21*t49;
	    t173 = t30*t47;
	    t174 = t172-t173;
	    t188 = t87*t147;
	    t204 = -t51;
	    t206 = -t174;
	    t211 = 0.3673*t55;
	    t212 = 0.3673*t54;
	    t216 = 0.3673*t50;
	    t217 = 0.3673*t48;
	    t221 = 0.3673*t172;
	    t222 = 0.3673*t173;
	    A[1] = -(-t145-t146+t149)*t28-(t148*t30+t20*t25+t24*t22)*t36+0.41*t145+0.41*t146-0.41*t149-(t53*t125+t162*t165+t46*t72)*t59-(t162*t174+t46*t169+t53*t51)*t75+0.41*t46*t33+0.41*t53*t31-0.41*t162*t21-(t95*t125*t89+t188*t165*t89+t88*t72*t89)*t105*t107-(t95*t125*t91+t188*t165*t91+t88*t72*t91)*t122*t107-(t188*t206+t95*t204+t88*t56)*t134*t107-t88*(t211+t212-0.41*t33)-t95*(t216+t217-0.41*t31)-t188*(-t221+t222+0.41*t21);
	    t228 = -t165;
	    A[2] = -(t46*t127+t162*t228+t53*t68)*t59-(t162*t206+t53*t204+t46*t56)*t75-(t88*t127*t89+t188*t228*t89+t95*t68*t89)*t105*t107-(t88*t127*t91+t188*t228*t91+t95*t68*t91)*t122*t107-(t88*t169+t188*t174+t95*t51)*t134*t107-t88*(-t212-t211)-t95*(-t217-t216)-t188*(-t222+t221);
	    A[3] = -(-t88*t115+t95*(-t204*t91+t110)-t188*t206*t91)*t105*t107-(t88*t98+t95*(t204*t89+t92)+t188*t206*t89)*t122*t107;


}
#endif






/**
 * This method computes the Jacobian matrix of the actuator #3.
 *
 * \param[in] theta Actual angles of each actuator.
 * \param[out] A The result. A jacobian matrix.
 *
 */
void  L7_CKinematics::CalculJ3(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float A[16])
{
	if(m_RobotType == ROBOT_CONFIG_MICO_4DOF_SERVICE)
	{
		CalculJ3Mico(theta, A);
	}
	if(m_RobotType == ROBOT_CONFIG_JACOV2_4DOF_SERVICE)
	{
		CalculJ3Jaco(theta, A);
	}

}

void L7_CKinematics::CalculJ3Jaco(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float A[16])
{
	  float t10;
	  float t2;
	  float t5;
	  float t8;

		float theta1 = theta[0];
		float theta2 = theta[1];
		float theta3 = theta[2];

	    t2 = SIN(theta1);
	    t5 = COS(theta2);
	    t8 = COS(theta1);
	    A[0] = 0.41*t2*t5-0.98E-2*t8;
	    t10 = SIN(theta2);
	    A[1] = -0.41*t8*t10;
	    A[2] = 0.0;
	    A[3] = 0.0;
	    A[4] = -0.41*t8*t5-0.98E-2*t2;
	    A[5] = -0.41*t2*t10;
	    A[6] = 0.0;
	    A[7] = 0.0;
	    A[8] = 0.0;
	    A[9] = 0.41*t5;
	    A[10] = 0.0;
	    A[11] = 0.0;
	    A[12] = 0.0;
	    A[13] = 0.0;
	    A[14] = 0.0;
	    A[15] = 0.0;
}


void L7_CKinematics::CalculJ3Mico(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float A[16])
{
	  float t10;
	  float t2;
	  float t5;
	  float t8;

		float theta1 = theta[0];
		float theta2 = theta[1];
		float theta3 = theta[2];

	    t2 = SIN(theta1);
	    t5 = COS(theta2);
	    t8 = COS(theta1);
	    A[0] = 0.29*t2*t5-0.7E-2*t8;
	    t10 = SIN(theta2);
	    A[1] = -0.29*t8*t10;
	    A[2] = 0.0;
	    A[3] = 0.0;
	    A[4] = -0.29*t8*t5-0.7E-2*t2;
	    A[5] = -0.29*t2*t10;
	    A[6] = 0.0;
	    A[7] = 0.0;
	    A[8] = 0.0;
	    A[9] = 0.29*t5;
	    A[10] = 0.0;
	    A[11] = 0.0;
	    A[12] = 0.0;
	    A[13] = 0.0;
	    A[14] = 0.0;
	    A[15] = 0.0;
}


/**
 * This method computes the Jacobian matrix of the actuator #3.
 *
 * \param[in] theta Actual angles of each actuator.
 * \param[out] A The result. A jacobian matrix.
 *
 */
void  L7_CKinematics::CalculJ4(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float A[16])
{
	if(m_RobotType == ROBOT_CONFIG_MICO_4DOF_SERVICE)
	{
		CalculJ4Mico(theta, A);
	}
	if(m_RobotType == ROBOT_CONFIG_JACOV2_4DOF_SERVICE)
	{
		CalculJ4Jaco(theta, A);
	}
}

void L7_CKinematics::CalculJ4Jaco(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float A[16])
{
	  float t10;
	  float t13;
	  float t14;
	  float t15;
	  float t18;
	  float t2;
	  float t21;
	  float t22;
	  float t24;
	  float t33;
	  float t35;
	  float t38;
	  float t41;
	  float t5;
	  float t6;
	  float t8;

		float theta1 = theta[0];
		float theta2 = theta[1];
		float theta3 = theta[2];

	    t2 = SIN(theta1);
	    t5 = COS(theta2);
	    t6 = t2*t5;
	    t8 = COS(theta1);
	    t10 = SIN(theta3);
	    t13 = SIN(theta2);
	    t14 = t2*t13;
	    t15 = COS(theta3);
	    A[0] = 0.41*t6-0.98E-2*t8-0.2073*t6*t10+0.2073*t14*t15;
	    t18 = t8*t13;
	    t21 = 0.2073*t18*t10;
	    t22 = t8*t5;
	    t24 = 0.2073*t22*t15;
	    A[1] = -0.41*t18+t21+t24;
	    A[2] = -t24-t21;
	    A[3] = 0.0;
	    A[4] = -0.41*t22-0.98E-2*t2+0.2073*t22*t10-0.2073*t18*t15;
	    t33 = 0.2073*t14*t10;
	    t35 = 0.2073*t6*t15;
	    A[5] = -0.41*t14+t33+t35;
	    A[6] = -t35-t33;
	    A[7] = 0.0;
	    A[8] = 0.0;
	    t38 = 0.2073*t5*t10;
	    t41 = 0.2073*t13*t15;
	    A[9] = 0.41*t5-t38+t41;
	    A[10] = -t41+t38;
	    A[11] = 0.0;
	    A[12] = 0.0;
	    A[13] = 0.0;
	    A[14] = 0.0;
	    A[15] = 0.0;


}


void L7_CKinematics::CalculJ4Mico(float theta[NB_ACTUATOR_MAX_HIGH_LEVEL], float A[16])
{

	float theta1 = theta[0];
	float theta2 = theta[1];
	float theta3 = theta[2];


	  float t10;
	  float t13;
	  float t14;
	  float t15;
	  float t18;
	  float t2;
	  float t21;
	  float t22;
	  float t24;
	  float t33;
	  float t35;
	  float t38;
	  float t41;
	  float t5;
	  float t6;
	  float t8;

	    t2 = SIN(theta1);
	    t5 = COS(theta2);
	    t6 = t2*t5;
	    t8 = COS(theta1);
	    t10 = SIN(theta3);
	    t13 = SIN(theta2);
	    t14 = t2*t13;
	    t15 = COS(theta3);
	    A[0] = 0.29*t6-0.7E-2*t8-0.1233*t6*t10+0.1233*t14*t15;
	    t18 = t8*t13;
	    t21 = 0.1233*t18*t10;
	    t22 = t8*t5;
	    t24 = 0.1233*t22*t15;
	    A[1] = -0.29*t18+t21+t24;
	    A[2] = -t24-t21;
	    A[3] = 0.0;
	    A[4] = -0.29*t22-0.7E-2*t2+0.1233*t22*t10-0.1233*t18*t15;
	    t33 = 0.1233*t14*t10;
	    t35 = 0.1233*t6*t15;
	    A[5] = -0.29*t14+t33+t35;
	    A[6] = -t35-t33;
	    A[7] = 0.0;
	    A[8] = 0.0;
	    t38 = 0.1233*t5*t10;
	    t41 = 0.1233*t13*t15;
	    A[9] = 0.29*t5-t38+t41;
	    A[10] = -t41+t38;
	    A[11] = 0.0;
	    A[12] = 0.0;
	    A[13] = 0.0;
	    A[14] = 0.0;
	    A[15] = 0.0;
}
