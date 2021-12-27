
#include "Movable.h"
#include <iostream>
Movable::Movable()
{
	Tout = Eigen::Affine3d::Identity();
	Tin = Eigen::Affine3d::Identity();
	//assignmet3
	//Rx = Eigen::Matrix4d::Identity();
	//Rz = Eigen::Matrix4d::Identity();
}

Movable::Movable(const Movable& mov)
{
	Tout = mov.Tout;
	Tin = mov.Tin;
}
/*
//assignment3
Eigen::Matrix4f Movable::MakeTrans()
{
	return (Tout * Tin).matrix();
}
*/

Eigen::Matrix4f Movable::MakeTransScale()
{
	return (Tout.matrix()*Tin.matrix()).cast<float>();
}

Eigen::Matrix4d Movable::MakeTransScaled()
{
	return (Tout.matrix() * Tin.matrix());
}

Eigen::Matrix4d Movable::MakeTransd()
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.col(3) << Tin.translation(), 1;

	return (Tout.matrix() * mat);
}

void Movable::MyTranslate(Eigen::Vector3d amt, bool preRotation)
{
	
	if(preRotation)
		Tout.pretranslate(amt);
	else
		Tout.translate(amt);
}
void Movable::TranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt)
{
	Tout.pretranslate(rot.transpose()*amt);
}
/*from lecture
void Movable::SetCenterOfRotation(Eigen::Vector3d amt) {
	Tout.pretranslate(amt);
	Tin.pretranslate(-amt);
}
*/
//assignment3

void Movable::SetCenterOfRotation(Eigen::Vector3d amt) {
	Tin.translate(-amt);
	Tout.pretranslate(amt);
}

//angle in radians
void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, rotAxis.normalized()));
}

//assignment 3
void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle, bool self)
{
	Eigen::Matrix3d mat = Tout.rotation().matrix();
	mat.transposeInPlace();
	if (self)
		Tout.rotate(Eigen::AngleAxisd(angle, rotAxis));
	else
		Tout.rotate(Eigen::AngleAxisd(angle, mat * rotAxis));
}



	void Movable::RotateInSystem(Eigen::Vector3d rotAxis, double angle)
{
		Tout.rotate(Eigen::AngleAxisd(angle, Tout.rotation().transpose() * rotAxis.normalized()));
}

void Movable::MyRotate(const Eigen::Matrix3d& rot)
{
	Tout.rotate(rot);
}

void Movable::MyScale(Eigen::Vector3d amt)
{
	Tin.scale(amt);
}


Eigen::Vector3d Movable::GetCenterOfRotation()
{
	return -Tin.translation();
}






























//void Movable::TranslateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d amt, bool preRotation)
//{
//	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * amt; //transpose instead of inverse
//	MyTranslate(v, preRotation);
//}
//
void Movable::RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle)
{
	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * rotAxis; //transpose instead of inverse
	MyRotate(v.normalized(), angle);
}


//void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
//{
//	Tout.pretranslate(Tout.rotation().matrix().block<3, 3>(0, 0) * amt);
//	Tin.translate(-amt);
//}
//
//Eigen::Vector3d Movable::GetCenterOfRotation()
//{
//	return Tin.translation();
//}
