#include <nori/integrator.h>
#include <nori/scene.h>

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

		Color3f radiance(0.f);
		Point3f p = its.p;
		Vector3f wi = (m_lightPosition - p).normalized();
		float V = 1.f;
		if (scene->rayIntersect(Ray3f(p + wi * (1e-5), wi), its))
			V = 0.f;
		float cosTheta = its.shFrame.n.dot(wi); // same with its.shFrame.cosTheta(its.shFrame.toLocal(wi)) 
		radiance += V * (m_lightEnerry / (4 * pow(M_PI, 2))) * std::max(0.f, cosTheta) / (p - m_lightPosition).dot(p - m_lightPosition);
		return radiance;
		
		/* Find the surface that is visible in the requested direction */
		//Intersection its;
		//if (!scene->rayIntersect(ray, its))
		//	return Color3f(0.0f);

		///* Return the component-wise absolute
		//   value of the shading normal as a color */
		//Normal3f n = its.shFrame.n.cwiseAbs();
		//return Color3f(n.x(), n.y(), n.z());
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