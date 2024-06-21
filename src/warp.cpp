/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

float tent(float x) {
	return x < 0.5f ? sqrt(2.0f * x) - 1.0f : 1.0f - sqrt(2.0f - 2.0f * x);
}
Point2f Warp::squareToTent(const Point2f& sample) {
	return Point2f(tent(sample.x()), tent(sample.y()));
}
float tentPdf(float t) {
	return t >= -1 && t <= 1 ? 1 - abs(t) : 0;
}
float Warp::squareToTentPdf(const Point2f& p) {
	return tentPdf(p.x()) * tentPdf(p.y());
}

Point2f Warp::squareToUniformDisk(const Point2f& sample) {
	return Point2f(sqrt(sample.x()) * cos(2 * M_PI * sample.y()), sqrt(sample.x()) * sin(2 * M_PI * sample.y()));
}

float Warp::squareToUniformDiskPdf(const Point2f& p) {
	return (p.norm() < 1) * INV_PI;
}


Vector3f Warp::squareToUniformTriangle(const Point2f& sample) {
	float su1 = sqrtf(sample.x());
	float u = 1.f - su1, v = sample.y() * su1;
	return Vector3f(u, v, 1.f - u - v);
}

Vector3f Warp::squareToUniformSphere(const Point2f& sample)
{
	float xi1 = sample.x(), xi2 = sample.y();
	float x, y, z, phi, cos_theta, sin_theta;
	phi = 2 * M_PI * xi1;
	cos_theta = 1 - 2 * xi2;
	sin_theta = sqrt(1 - pow(cos_theta, 2));
	x = sin_theta * cos(phi);
	y = sin_theta * sin(phi);
	z = cos_theta;
	Point3f squareToTent_sample(x, y, z);
	return squareToTent_sample;
}

float Warp::squareToUniformSpherePdf(const Vector3f& v)
{
	if (fabs(1.f - sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2))) > 1e-5)
		return 0;
	return 1.f / (4.f * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f& sample)
{
	float xi1 = sample.x(), xi2 = sample.y();
	float x, y, z, phi, cos_theta, sin_theta;
	phi = 2 * M_PI * xi1;
	cos_theta = 1 - xi2;
	sin_theta = sqrt(1 - pow(cos_theta, 2));
	x = sin_theta * cos(phi);
	y = sin_theta * sin(phi);
	z = cos_theta;
	Point3f squareToTent_sample(x, y, z);
	return squareToTent_sample;
}

float Warp::squareToUniformHemispherePdf(const Vector3f& v)
{
	if (fabs(1.f - sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2))) > 1e-5 || (v.z() < 0))
		return 0;
	return 1.f / (2.f * M_PI);
}

Vector3f Warp::squareToCosineHemisphere(const Point2f& sample)
{
	float xi1 = sample.x(), xi2 = sample.y();
	float x, y, z, phi, cos_theta, sin_theta;
	phi = 2 * M_PI * xi1;
	sin_theta = sqrt(xi2);
	cos_theta = sqrt(1 - xi2);
	x = sin_theta * cos(phi);
	y = sin_theta * sin(phi);
	z = cos_theta;
	Point3f squareToTent_sample(x, y, z);
	return squareToTent_sample;
}

float Warp::squareToCosineHemispherePdf(const Vector3f& v)
{
	if (fabs(1.f - sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2))) > 1e-5 || (v.z() < 0))
		return 0;
	return v.z() / M_PI;
}

Vector3f Warp::squareToBeckmann(const Point2f& sample, float alpha)
{
	float xi1 = sample.x(), xi2 = sample.y();
	float x, y, z;
	float tan2theta = -pow(alpha, 2) * log(1 - xi2);
	x = 1.f / (sqrt(1.f + 1.f / tan2theta)) * cos(2 * M_PI * xi1);
	y = 1.f / (sqrt(1.f + 1.f / tan2theta)) * sin(2 * M_PI * xi1);
	z = 1 / (sqrt(1 + tan2theta));
	Point3f squareToTent_sample(x, y, z);
	return squareToTent_sample;
}

float Warp::squareToBeckmannPdf(const Vector3f& v, float alpha)
{
	if ((v.z() <= 0))
		return 0; // must exclude when z=0, z be used as the denominator
	return exp(-(1.f - pow(v.z(), 2)) / (pow(alpha, 2) * pow(v.z(), 2))) / (pow(alpha, 2) * pow(v.z(), 3)) * INV_PI;
}
NORI_NAMESPACE_END
