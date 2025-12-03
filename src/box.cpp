#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN
class Box : public Shape {
public:
  Box(const PropertyList &props) {
    this->bounds[0] = Vector3f(0,0,0); // min corner
    this->bounds[1] = Vector3f(1,1,1); // max corner
  }

  void activate() {
    if (!m_bsdf) {
      /* If no material was assigned, instantiate a diffuse BRDF */
      m_bsdf = static_cast<BSDF *>(
          NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }
  }

  bool rayIntersect(Ray3f &ray, Intersection &its,
                    bool shadowRay = false) const {

    Normal3f n(1, 0, 0);
    int hitAxis = 0;
    float aux;
    float tmin = (bounds[0].x() - ray.o.x()) / ray.d.x();
    float tmax = (bounds[1].x() - ray.o.x()) / ray.d.x();

    if (tmin > tmax){
      aux = tmax;
      tmax = tmin;
      tmin = aux;
    }

    float tymin = (bounds[0].y() - ray.o.y()) / ray.d.y();
    float tymax = (bounds[1].y() - ray.o.y()) / ray.d.y();

    if (tymin > tymax){
      aux = tymax;
      tymax = tymin;
      tymin = aux;
    }

    if ((tmin > tymax) || (tymin > tmax)) return false;

    if (tymin > tmin) { tmin = tymin; hitAxis = 1; }

    if (tymax < tmax) tmax = tymax;

    float tzmin = (bounds[0].z() - ray.o.z()) / ray.d.z();
    float tzmax = (bounds[1].z() - ray.o.z()) / ray.d.z();

    if (tzmin > tzmax){
      aux = tzmax;
      tzmax = tzmin;
      tzmin = aux;
    }

    if ((tmin > tzmax) || (tzmin > tmax)) return false;

    if (tzmin > tmin) { tmin = tzmin; hitAxis = 2; }

    if (tzmax < tmax) tmax = tzmax;

    float t = (tmin > ray.mint) ? tmin : ray.mint;

    if (hitAxis == 0) n = (ray.d.x() > 0) ? Normal3f(-1,0,0) : Normal3f(1,0,0);
    if (hitAxis == 1) n = (ray.d.y() > 0) ? Normal3f(0,-1,0) : Normal3f(0,1,0);
    if (hitAxis == 2) n = (ray.d.z() > 0) ? Normal3f(0,0,-1) : Normal3f(0,0,1);

    if (shadowRay == true){
      return true;
    }
    updateRayAndHit(ray, its, t, n);
    return true;
      
  }

  /// Register a child object (e.g. a BSDF) with the mesh
  void addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
    case EBSDF:
      if (m_bsdf)
        throw NoriException(
            "Box: tried to register multiple BSDF instances!");
      m_bsdf = static_cast<BSDF *>(obj);
      break;

    case EEmitter: {
      Emitter *emitter = static_cast<Emitter *>(obj);
      if (m_emitter)
        throw NoriException(
            "Box: tried to register multiple Emitter instances!");
      m_emitter = emitter;
    } break;

    default:
      throw NoriException("Box::addChild(<%s>) is not supported!",
                          classTypeName(obj->getClassType()));
    }
  }

  std::string toString() const {
    return tfm::format(
        "Box[\n"
        "emitter = %s\n"
        "bsdf = %s\n"
        "]",
        (m_emitter) ? indent(m_emitter->toString()) : std::string("null"),
        (m_bsdf) ? indent(m_bsdf->toString()) : std::string("null"));
  }

private:
  void updateRayAndHit(Ray3f &ray, Intersection &its, float t,
                       const Normal3f &n) const {
    ray.maxt = its.t = t;
    its.p = ray(its.t);
    its.uv = Point2f(its.p.x(), its.p.z());
    its.bsdf = m_bsdf;
    its.emitter = m_emitter;
    its.geoFrame = its.shFrame = Frame(n);
  }

private:
  BSDF *m_bsdf = nullptr;       ///< BSDF of the surface
  Emitter *m_emitter = nullptr; ///< Associated emitter, if any
  Vector3f bounds[3];
};

NORI_REGISTER_CLASS(Box, "box");
NORI_NAMESPACE_END
