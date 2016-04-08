/* CS3242 3D Modeling and Animation
 * Programming Assignment II
 * School of Computing
 * National University of Singapore
 */
 
#ifndef  _BVH_H_
#define  _BVH_H_

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/quaternion.hpp"
#include "GL/glut.h"

#include <vector>
#include <string>
#include <map>

/**
 * The joint names
 * as in NUS mocap BVH file.
 */

// precision used
#define _BVH_PRECISION_ 6

// spine
#define _BVH_ROOT_JOINT_            "Hips"
#define _BVH_SPINE_JOINT_           "Spine"
#define _BVH_CHEST_JOINT_           "Spine1"
#define _BVH_NECK_JOINT_            "Neck"
#define _BVH_HEAD_JOINT_            "Head"

// left arm
#define _BVH_L_SHOULDER_JOINT_      "LeftShoulder"
#define _BVH_L_ARM_JOINT_           "LeftArm"
#define _BVH_L_FOREARM_JOINT_       "LeftForeArm"

// left hand
#define _BVH_L_HAND_JOINT_          "LeftHand"
#define _BVH_L_HANDEND_JOINT_       "L_Wrist_End"
#define _BVH_L_HANDTHUMB_JOINT_     "LeftHandThumb"

// right arm
#define _BVH_R_SHOULDER_JOINT_      "RightShoulder"
#define _BVH_R_ARM_JOINT_           "RightArm"
#define _BVH_R_FOREARM_JOINT_       "RightForeArm"

// right hand
#define _BVH_R_HAND_JOINT_          "RightHand"
#define _BVH_R_HANDEND_JOINT_       "R_Wrist_End"
#define _BVH_R_HANDTHUMB_JOINT_     "RightHandThumb"

// left leg
#define _BVH_L_THIGH_JOINT_         "LeftUpLeg"
#define _BVH_L_SHIN_JOINT_          "LeftLeg"
#define _BVH_L_FOOT_JOINT_          "LeftFoot"
#define _BVH_L_TOE_JOINT_           "LeftToeBase"

// right leg
#define _BVH_R_THIGH_JOINT_         "RightUpLeg"
#define _BVH_R_SHIN_JOINT_          "RightLeg"
#define _BVH_R_FOOT_JOINT_          "RightFoot"
#define _BVH_R_TOE_JOINT_           "RightToeBase"

enum  ChannelEnum
{
	X_ROTATION, Y_ROTATION, Z_ROTATION,
	X_POSITION, Y_POSITION, Z_POSITION
};

typedef unsigned int uint;

typedef struct{
	ChannelEnum type;
	uint index;
} CHANNEL;

typedef struct
{
	float x, y, z;
} OFFSET;

typedef struct JOINT JOINT;

struct RigidTransform {
    glm::vec3 translation;
    glm::quat quaternion;
};

struct JOINT
{
	std::string name;                     // joint name
	JOINT* parent;                        // joint parent	
	OFFSET offset;                        // offset data
	std::vector<CHANNEL*> channels;
	std::vector<JOINT*> children;         // joint's children	
	bool is_site;                         // if it is end site

    RigidTransform transform;             // transformation stored by a translation and a quaternion (for animation)

    glm::mat4 matrix;                     // transformation stored by 4x4 matrix (for reference only)
};

typedef struct
{
	unsigned int num_frames;              // number of frames
	unsigned int num_motion_channels;     // number of motion channels (of all joints per frame)
	float* data;                          // motion float data array
	float frame_time;                     // time per frame; FPS = 1/frame_time
} MOTION;

class BVH
{
private:
	JOINT* rootJoint;
	MOTION motionData;
	bool load_success;
	std::map<std::string,JOINT*> nameToJoint;
	std::vector<JOINT*> jointList;             // this list stores all pointers to joints
	std::vector<CHANNEL*> channels;

public:
	BVH();
	BVH(const char* filename);
	~BVH();

private:
	// write a joint info to bvh hierarchy part
	void writeJoint(JOINT* joint, std::ostream& stream, int level);	

	// clean up stuff, used in destructor
	void clear();

public:
	/************************************************************************** 
	 * functions for basic I/O - only loading is needed
	 **************************************************************************/
	
    // load a bvh file
	void load(const std::string& filename);  	

	// is the bvh file successfully loaded? 
	bool IsLoadSuccess() const { return load_success; }

public:
	/************************************************************************** 
	 * functions to retrieve bvh data 
	 **************************************************************************/
	
    // get root joint
	JOINT* getRootJoint(){ return rootJoint;}
	
    // get the JOINT pointer from joint name 
	JOINT* getJoint(std::string name);	
	
    // get the pointer to mation data at frame_no
	// NOTE: frame_no is treated as frame_no % total_frames
	float* getMotionDataPtr(int frame_no);
	
    // get a pointer to the root joint 
	const JOINT* getRootJoint() const { return rootJoint; }
	
    // get the list of the joint
	const std::vector<JOINT*> getJointList() const {return jointList; }
	
    // get the number of frames 
	unsigned getNumFrames() const { return motionData.num_frames; }
	
    // get time per frame
	float getFrameTime() const { return motionData.frame_time; }	
	
public:
    // calculate JOINT's transformation for specified frame using matrix	
	void matrixMoveTo(unsigned frame, float scale);

    // calculate JOINT's transformation for specified frame using quaternion
    void quaternionMoveTo(unsigned frame, float scale);

private:    
	void quaternionMoveJoint(JOINT* joint, float* mdata, float scale);	
    void matrixMoveJoint(JOINT* joint, float* mdata, float scale);	    	

};

#endif
