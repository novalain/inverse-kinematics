/* CS3242 3D Modeling and Animation
 * Programming Assignment II
 * School of Computing
 * National University of Singapore
 */
 
#ifndef _BVH_ANIMATOR_H_
#define _BVH_ANIMATOR_H_

#include "BVH.h"

class BVHAnimator
{
private:
	BVH* _bvh;

	// pointers used for rendering - 21 pointers
	JOINT *head, *neck, *chest, *spine, *hip;
	JOINT *lshldr, *larm, *lforearm, *lhand;
	JOINT *rshldr, *rarm, *rforearm, *rhand;
	JOINT *lupleg, *lleg, *lfoot, *ltoe;
	JOINT *rupleg, *rleg, *rfoot, *rtoe;
	std::vector<glm::vec4> vertices;
	std::vector<GLshort>   indices;
		
	// sets all pointers used for rendering
	void setPointers();

public:
	BVHAnimator(BVH* bvh);

	void setBVH(BVH* bvh) {_bvh = bvh;}
		
	// draw the character at frame_no
	void renderFigure( int frame_no, float scale, int flag);	

	// render the character skeleton
	void renderSkeleton( const JOINT* root, const float* data, float scale = 1.0f );

	// render the character joints with transformation done by OpenGL matrix stack
	void renderJointsGL(const JOINT* root, const float* data, float scale = 1.0f);

    // render the character joints with transformation done by our own matrix 
    void renderJointsMatrix(int frame, float scale = 1.0f);

	// render the character joints with transformation done by our own quaternion and translation vector
	void renderJointsQuaternion(int frame, float scale = 1.0f);

	// draw the mannequin
	void renderMannequin(int frame_no, float scale = 1.0f);	

	/**
	 * Calculate and update joint's motion data by solving IK for left arm 
	 * joints to position character's LeftHand to reach target (x, y, z).
	 * Joints: LeftArm 3DOF, LeftForeArm 1DOF
	 */
	void solveLeftArm(int frame_no, float scale, float x, float y, float z);
};

#endif