#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN


class NormalIntegrator : public Integrator {
public:
	NormalIntegrator(const PropertyList& props) {
		m_lightPosition = props.getPoint("position");
		m_lightEnerry = props.getColor("energy");

		//std::cout << "Parameter value was : " << m_myProperty << std::endl;
	}

	/// Compute the radiance value for a given ray. Just return green here
	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		Color3f color(0.f);
		Point3f p = its.p;
		Point2f sample = sampler->next2D();
		Vector3f wi = Warp::squareToCosineHemisphere(sample);
		float cosTheta = wi.z();
		float pdf = Warp::squareToCosineHemispherePdf(wi);
		wi = its.shFrame.toWorld(wi);
		float V = 1.f;
		if (scene->rayIntersect(Ray3f(p + wi * (1e-5), wi), its))
			V = 0.f;
		color += V * cosTheta * INV_PI / pdf;
		// if (fabs(1 - cosTheta * INV_PI / pdf) > 1e-5)
		//     std::cout << "dif" << std::endl;
		return color;
	}

	/// Return a human-readable description for debugging purposes
	std::string toString() const {
		return tfm::format(
			"NormalIntegrator[\n"
			"]"
		);
	}
protected:
	Point3f m_lightPosition;
	Color3f m_lightEnerry;
};

NORI_REGISTER_CLASS(NormalIntegrator, "simple");
NORI_NAMESPACE_END