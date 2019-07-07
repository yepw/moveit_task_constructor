#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gtest/gtest.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

using namespace moveit::task_constructor;

void spawnObject(){
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "base_link";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x= 0.0;
	o.primitive_poses[0].position.y= 0.8;
	o.primitive_poses[0].position.z= 0.2;
	o.primitive_poses[0].orientation.w= 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0]= 0.23;
	o.primitives[0].dimensions[1]= 0.03;
	psi.applyCollisionObject(o);
}

TEST(UR5, pick) {
	Task t;

	Stage* initial_stage = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	initial_stage = initial.get();
	t.add(std::move(initial));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{"manipulator", pipeline}, {"gripper", pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));

	// grasp generator
	auto grasp_generator = new stages::GenerateGraspPose("generate grasp pose");
	grasp_generator->setAngleDelta(.2);
	grasp_generator->setPreGraspPose("open");
	grasp_generator->setGraspPose("closed");
	grasp_generator->setMonitoredStage(initial_stage);

	auto grasp = std::make_unique<stages::SimpleGrasp>(std::unique_ptr<MonitoringGenerator>(grasp_generator));
	Eigen::Isometry3d eigen;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0,0,0.3));
	transform.setRotation(tf::createQuaternionFromRPY(-M_PI/2, 0, 0));
	tf::transformTFToEigen(transform,eigen);
	grasp->setIKFrame(eigen, "fts_toolside");
	grasp->setMaxIKSolutions(8);

	auto pick = std::make_unique<stages::Pick>(std::move(grasp));
	pick->setProperty("eef", std::string("gripper"));
	pick->setProperty("object", std::string("object"));
	geometry_msgs::TwistStamped approach;
	approach.header.frame_id = "fts_toolside";
	approach.twist.linear.z = 1.0;
	pick->setApproachMotion(approach, 0.03, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "world";
	lift.twist.linear.z = 1.0;
	pick->setLiftMotion(lift, 0.03, 0.05);

	t.add(std::move(pick));

	//*************************
	
	Stage* object_grasped= nullptr;
    auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
    // stage->attachObject("object", "fts_toolside");
    object_grasped= stage.get();
    t.add(std::move(stage));
 
	auto connect2 = std::make_unique<stages::Connect>("connect2", planners);
	connect2->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect2));

	auto ungrasp_generator = new stages::GeneratePlacePose("generate place pose");
	ungrasp_generator->properties().configureInitFrom(Stage::PARENT);
	geometry_msgs::PoseStamped placeMsgPose;
	placeMsgPose.header.frame_id = "world";
	placeMsgPose.pose.position.x = 0.2;
	placeMsgPose.pose.position.y = 0.8;
	placeMsgPose.pose.position.z = 0.2;
	placeMsgPose.pose.orientation.w = 1.0;
	ungrasp_generator->setPose(placeMsgPose);
	ungrasp_generator->setMonitoredStage(object_grasped);
	geometry_msgs::PoseStamped ik_pose;
	ik_pose.header.frame_id = "fts_toolside";
	tf::poseEigenToMsg(eigen, ik_pose.pose);
	// ungrasp_generator->setProperty("ik_frame", ik_pose);
	ungrasp_generator->setProperty("object", std::string("object"));

	auto ungrasp = std::make_unique<stages::SimpleUnGrasp>(std::unique_ptr<MonitoringGenerator>(ungrasp_generator));
	ungrasp->setMaxIKSolutions(8);
	ungrasp->setIKFrame("fts_toolside");
	ungrasp->setProperty("pregrasp", std::string("open"));
	

	auto place = std::make_unique<stages::Place>(std::move(ungrasp));
	place->setProperty("eef", std::string("gripper"));
	place->setProperty("object", std::string("object"));
	geometry_msgs::TwistStamped place_approach;
	place_approach.header.frame_id = "world";
	place_approach.twist.linear.z = -1;
	place->setPlaceMotion(place_approach, 0.03, 0.1);


	geometry_msgs::TwistStamped place_retract;
	place_retract.header.frame_id = "fts_toolside";
	place_retract.twist.linear.z = -1.0;
	place->setRetractMotion(place_retract, 0.03, 0.05);

	t.add(std::move(place));
	
	try {
		spawnObject();
		t.plan();
	} catch (const InitStageException &e) {
		ADD_FAILURE() << "planning failed with exception" << std::endl << e << t;
	}
	
	auto solutions = t.solutions().size();
	// t.execute(*t.solutions().front());
	t.publishAllSolutions();
	EXPECT_GE(solutions, 30u);
	EXPECT_LE(solutions, 60u);

	
}

int main(int argc, char** argv){
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "ur5");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// wait some time for move_group to come up
	ros::WallDuration(5.0).sleep();
	return RUN_ALL_TESTS();
}
