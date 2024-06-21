#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class WhittedIntegrator : public Integrator {
public:
	WhittedIntegrator(const PropertyList& props) {}


	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		const Emitter* emitter = scene->getRandomEmitter(sampler->next1D());

		EmitterQueryRecord lRecR;
		lRecR.ref = its.p;
		Color3f Li = emitter->sample(lRecR, sampler->next2D()) * scene->getLights().size();

		float cosTheta = Frame::cosTheta(its.shFrame.toLocal(lRecR.wi));

		BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(lRecR.wi), ESolidAngle);
		bRec.uv = its.uv;
		Color3f f = its.mesh->getBSDF()->eval(bRec);

		// emitted
		Color3f Le = 0;
		if (its.mesh->isEmitter()) {
			EmitterQueryRecord lRecE(ray.o, its.p, its.shFrame.n);
			Le = its.mesh->getEmitter()->eval(lRecE);
		}

		// check if shadow ray is occluded
		if (scene->rayIntersect(lRecR.shadowRay, its))
			Li = 0;

		return Le + Li * f * std::max(0.f, cosTheta);
	}
	std::string toString() const {
		return "WhittedIntegrator[]";
	}
};
NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END