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

#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList& propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    virtual Color3f eval(const BSDFQueryRecord&) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    virtual float pdf(const BSDFQueryRecord&) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    virtual Color3f sample(BSDFQueryRecord& bRec, const Point2f& sample) const override {

        if (fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR) > sample.x()) {
            // Reflection in local coordinates
            bRec.wo = Vector3f(
                -bRec.wi.x(),
                -bRec.wi.y(),
                bRec.wi.z()
            );
            bRec.eta = 1.0f;
        }
        else {
            bRec.eta = m_extIOR / m_intIOR;
            Vector3f n = Vector3f(0, 0, 1);
            if (Frame::cosTheta(bRec.wi) <= 0.0f) {
                bRec.eta = 1 / bRec.eta;
                n *= -1;
            }

            bRec.wo = (-bRec.eta * (bRec.wi - (bRec.wi.dot(n) * n)) - n * sqrt(1 - pow(bRec.eta, 2) * (1 - pow(bRec.wi.dot(n), 2)))).normalized();
            bRec.eta = m_intIOR / m_extIOR;
        }

        bRec.measure = EDiscrete;
        return Color3f(1.0f);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
