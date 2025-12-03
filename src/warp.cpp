/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
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

Point2f Warp::squareToTent(const Point2f &sample) {
    float xinv, yinv;
    if (sample.x() < 0.5f){
        xinv = -1 + std::sqrt(2*sample.x());
    }
    else {
        xinv = 1 - std::sqrt(2*(1-sample.x()));
    }
    if (sample.y() < 0.5f){
        yinv = -1 + std::sqrt(2*sample.y());
    }
    else {
        yinv = 1 - std::sqrt(2*(1-sample.y()));  // Fixed: was sample.x()
    }
    return Point2f(xinv, yinv);
}

float Warp::squareToTentPdf(const Point2f &p) {
    return (1 - std::abs(p.x())) * (1 - std::abs(p.y()));
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = std::sqrt(sample.x());
    float theta = 2.0f * M_PI * sample.y();
    
    float x = r * std::cos(theta);
    float y = r * std::sin(theta);


    return Point2f(x,y);
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    float dist = std::sqrt(p.x() * p.x() + p.y() * p.y());
    if (dist <= 1.0f){
        return 1.0f/M_PI;
    }
    return 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float z = 1.0f * 2.0f * sample.x();
    float theta = 2.0f * M_PI * sample.y();
    
    float r = std::sqrt(1.0f - z * z);
    float x = r * std::cos(theta);
    float y = r * std::sin(theta);

    return Vector3f(x,y,z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return 1.0f/(4.0f * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float z = sample.x();
    float theta = 2.0f * M_PI * sample.y();
    
    float r = std::sqrt(1.0f - z * z);
    float x = r * std::cos(theta);
    float y = r * std::sin(theta);
    
    return Vector3f(x, y, z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    if (v.z() < 0.0f) {
        return 0.0f;
    }
    return 1.0f / (2.0f * M_PI);
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float r = std::sqrt(sample.x());
    float theta = 2.0f * M_PI * sample.y();
    
    float x = r * std::cos(theta);
    float y = r * std::sin(theta);
    float z = std::sqrt(1.0f - x * x - y * y); 
    
    return Vector3f(x, y, z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    if (v.z() < 0.0f) {
        return 0.0f;
    }
    return v.z() / M_PI;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
