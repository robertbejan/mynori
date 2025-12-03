#include <nori/shape.h>
#include <nori/transform.h>

NORI_NAMESPACE_BEGIN

class Instance : public Shape {
private:
    Shape *m_shape = nullptr;
    Transform m_transform;

public:
    Instance(const PropertyList &props) {
        m_transform = props.getTransform("toWorld");
    }

    void activate() override {
        if (!m_shape)
            throw NoriException("instance: no underlying shape provided!");
    }

    bool rayIntersect(Ray3f &ray, Intersection &its,
                      bool shadowRay = false) const override {
        if (!m_shape)
            return false;

        Ray3f localRay = m_transform.inverse()*ray;
        
        bool hit = m_shape->rayIntersect(localRay, its, shadowRay);
        if (!hit)
            return false;

        its.p = m_transform.operator*(its.p);

        Normal3f n = m_transform.operator*(its.shFrame.n);
        its.shFrame = Frame(n);

        ray.maxt = localRay.maxt;

        return true;

        // Ray3f localRay = m_transform.inverse()*ray;
        // bool hit = m_shape->rayIntersect(localRay, its, shadowRay);
        // if (hit){
        //     its.p = m_transform*its.p;
        //     its.shFrame.n = m_transform*its.shFrame.n;
        //     ray.maxt = localRay.maxt;
        // }


    }

    void addChild(NoriObject *obj) override {
        switch (obj->getClassType()) {
        case EShape:
            if (m_shape)
                throw NoriException(
                    "instance: tried to register multiple Shape instances!");
            m_shape = static_cast<Shape *>(obj);
            break;

        default:
            throw NoriException("instance::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const override {
        return tfm::format(
            "instance[\n"
            "  shape = %s\n"
            "  toWorld = %s\n"
            "  emitter = %s\n"
            "  bsdf = %s\n"
            "]",
            (m_shape) ? indent(m_shape->toString()) : std::string("null"),
            indent(m_transform.toString()));}
};

NORI_REGISTER_CLASS(Instance, "instance");
NORI_NAMESPACE_END
