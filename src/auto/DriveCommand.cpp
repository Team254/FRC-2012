#include "auto/DriveCommand.h"

DriveCommand::DriveCommand(Drive* drive, double distance) {
	drive_ = drive;
	distanceGoal_ = distance;
	leftPid_ = new Pid(.1, 0, 0);
	leftPid_->SetGoal(distanceGoal_);
	rightPid_ = new Pid(.1, 0, 0);
	rightPid_->SetGoal(distanceGoal_);
	distanceGoal_ = distance;
}

void DriveCommand::Initialize() {
	drive_->ResetEncoders();
}

bool DriveCommand::Run() {
	double currLeftDist = drive_->GetLeftEncoderDistance();
	//Bhargee: have to change this, we only had 1 encoder
	double currRightDist = drive_->GetLeftEncoderDistance();
	if(currLeftDist == distanceGoal_ && currRightDist == distanceGoal_) 
		return true;
	double leftPwr = leftPid_->Update(currLeftDist);
	double rightPwr = rightPid_->Update(currRightDist);
	drive_->SetLinearPower(leftPwr, rightPwr);
	return false;
}

DriveCommand::~DriveCommand() {
	delete leftPid_;
	delete rightPid_;
	delete drive_;
}
