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

#include <nori/accel.h>
#include <nori/timer.h>
#include <tbb/tbb.h>
#include <Eigen/Geometry>
#include <atomic>
#include <stack>
#include <set>

/*
 * =======================================================================
 *   WARNING    WARNING    WARNING    WARNING    WARNING    WARNING
 * =======================================================================
 *   Remember to put on SAFETY GOGGLES before looking at this file. You
 *   are most certainly not expected to read or understand any of it.
 * =======================================================================
 */

NORI_NAMESPACE_BEGIN
struct OctreeNode {
public:
  OctreeNode(const BoundingBox3f& bbox):bbox(bbox) {}
  virtual ~OctreeNode() {
    for( auto child : children) {
      delete child;
    }
  }
  
  virtual bool is_leaf() const {
    return false;
  }

  virtual unsigned int nbTriangles() const {
    return 0;
  }

  virtual const Triangle* getTriangles() const {
    return nullptr;
  }

public:
  using NodePtr = OctreeNode*;
  BoundingBox3f bbox;
  static const uint32_t triangles_per_leaf = 10;
  NodePtr children[8] = {nullptr};
  bool m_useOctree = false; // Change this to set to Brute Force or Octree
};

struct OctreeLeaf : public OctreeNode {
  OctreeLeaf(const BoundingBox3f& bbox, const std::vector<Triangle>& triangles): OctreeNode(bbox), size(triangles.size()), triangles(new Triangle[size]) {
    for(unsigned int k=0; k<size; ++k) {
      this->triangles[k] = triangles[k];
    }
  }

  ~OctreeLeaf() {
    delete[] triangles;
  }

  bool is_leaf() const override {
    return true;
  }

  unsigned int nbTriangles() const override {
    return size;
  }

  const Triangle* getTriangles() const override {
    return triangles;
  }

public:
  unsigned int size;
  Triangle *triangles;
};


std::vector<BoundingBox3f> subdivideBoundingBox(const BoundingBox3f &box) {
    std::vector<BoundingBox3f> children(8);
    Point3f min = box.min;
    Point3f max = box.max;
    Point3f center = box.getCenter();

    // Define the 8 sub-boxes (octants)
    // Each child box is defined by min/max corners based on center splits
    children[0] = BoundingBox3f{min, center};
    children[1] = BoundingBox3f{Point3f(center.x(), min.y(), min.z()), Point3f(max.x(), center.y(), center.z())};
    children[2] = BoundingBox3f{Point3f(min.x(), center.y(), min.z()), Point3f(center.x(), max.y(), center.z())};
    children[3] = BoundingBox3f{Point3f(center.x(), center.y(), min.z()), Point3f(max.x(), max.y(), center.z())};
    children[4] = BoundingBox3f{Point3f(min.x(), min.y(), center.z()), Point3f(center.x(), center.y(), max.z())};
    children[5] = BoundingBox3f{Point3f(center.x(), min.y(), center.z()), Point3f(max.x(), center.y(), max.z())};
    children[6] = BoundingBox3f{Point3f(min.x(), center.y(), center.z()), Point3f(center.x(), max.y(), max.z())};
    children[7] = BoundingBox3f{center, max};

    return children;
}

OctreeNode*
Accel::build(const BoundingBox3f& bbox, const Triangles & triangles){
  if (triangles.size() < OctreeNode::triangles_per_leaf)
  {
    return new OctreeLeaf(bbox,triangles);
  }

  Triangles triangle_list[8];
  std::vector<BoundingBox3f> childrenBoxes = subdivideBoundingBox(bbox);
  for (auto idx : triangles) {
    BoundingBox3f tri_bbox = getBoundingBox(idx);
    for (int i = 0; i < 8; ++i) {
      if (tri_bbox.overlaps(childrenBoxes[i])) {
        triangle_list[i].push_back(idx);
        //! beware infinite recursion if "list[i]==triangles"
        if (triangle_list[i].size() == triangles.size())
          return new OctreeLeaf(bbox,triangles);
      }
    }
  }

  OctreeNode *node = new OctreeNode(bbox);
  for (int i = 0; i < 8; ++i)
    node->children[i] = build(childrenBoxes[i], triangle_list[i]);
  return node;
}


void Accel::addMesh(Mesh *mesh) {
    m_meshes.push_back(mesh);
    uint32_t last_triangle = m_meshOffset.back();
    for(uint32_t triangle=last_triangle; triangle<last_triangle+mesh->getTriangleCount(); ++triangle) {
      m_triangles.push_back(triangle);
    }
    m_meshOffset.push_back(last_triangle + mesh->getTriangleCount());
    m_bbox.expandBy(mesh->getBoundingBox());
}

void Accel::clear() {
    for (auto mesh : m_meshes)
        delete mesh;
    m_meshes.clear();
    m_meshOffset.clear();
    m_meshOffset.push_back(0u);
    m_bbox.reset();
    m_meshes.shrink_to_fit();
    m_meshOffset.shrink_to_fit();
    delete m_octree;
}
void collectTriangles(const OctreeNode* node, std::set<Triangle>& all_triangles) {
    if (!node)
        return;

    if (node->is_leaf()) {
        const Triangle* tris = node->getTriangles();
        unsigned int count = node->nbTriangles();
        for (unsigned int i = 0; i < count; ++i) {
            all_triangles.insert(tris[i]);
        }
    } else {
        for (int i = 0; i < 8; ++i) {
            collectTriangles(node->children[i], all_triangles);
        }
    }
}


void Accel::activate() {
    uint32_t size  = getTriangleCount();
    if (size == 0)
        return;
    cout << "Constructing an Octree (" << m_meshes.size()
        << (m_meshes.size() == 1 ? " mesh, " : " meshes, ")
        << size << " triangles) .. ";
    cout.flush();
    Timer timer;

    m_octree = build(m_bbox, m_triangles); 
    std::set<Triangle> all_triangles;
    {
      collectTriangles(m_octree, all_triangles);
    }
    cout << "done (took " << timer.elapsedString() << ")." << endl;
    if (all_triangles.size() == m_triangles.size()) {
      cout << "All triangles in the octree [OK]" << endl;
    }
    else {
      cout << "All triangles in the octree [FAILED]" << endl;
    }

}

bool Accel::rayIntersect(Ray3f &_ray, Intersection &its, bool shadowRay) const {

    /* Use an adaptive ray epsilon and check maxt/mint as before */
    Ray3f ray(_ray);
    if (ray.mint == Epsilon)
        ray.mint = std::max(ray.mint, ray.mint * ray.o.array().abs().maxCoeff());
    if (ray.maxt < ray.mint)
        return false;

    bool foundIntersection = false;
    const Mesh *hitmesh   = nullptr;
    uint32_t f = 0;

    // 💡 CHOICE MECHANISM: Check the flag to choose the method
    if (m_useOctree && m_octree) {
        // --- OCTREE TRAVERSAL (Accelerated) ---
        
        // Stack for octree traversal
        std::stack<const OctreeNode*> stack;
        stack.push(m_octree);

        // Current closest intersection
        float closest_t = ray.maxt;
        foundIntersection = false;

        while (!stack.empty()) {
            const OctreeNode* node = stack.top();
            stack.pop();

            if (!node)
                continue;

            float tnear, tfar;
            // Skip nodes that the ray does not intersect or are farther than current hit
            if (!node->bbox.rayIntersect(ray, tnear, tfar) || tnear > closest_t)
                continue;

            if (node->is_leaf()) {
                // Test intersection with all triangles in the leaf node
                const Triangle* tris = node->getTriangles();
                unsigned int count = node->nbTriangles();

                for (unsigned int i = 0; i < count; ++i) {
                    uint32_t idx = tris[i];
                    uint32_t local_idx = idx;
                    auto mesh = getMesh(findMesh(local_idx));

                    float u, v, t;
                    if (mesh->rayIntersect(idx, ray, u, v, t)) {
                        if (shadowRay)
                            return true;

                        // Update closest intersection
                        if (t < closest_t) {
                            closest_t = t;
                            ray.maxt = its.t = t;
                            its.uv = Point2f(u, v);
                            f = idx;
                            hitmesh = mesh;
                            foundIntersection = true;
                        }
                    }
                }
            } else {
                // Collect child nodes that the ray intersects, along with their entry distances
                struct ChildHit {
                    const OctreeNode* node;
                    float tnear;
                };
                std::vector<ChildHit> hitChildren;

                for (int i = 0; i < 8; ++i) {
                    const OctreeNode* child = node->children[i];
                    if (!child)
                        continue;

                    float cn, cf;
                    if (child->bbox.rayIntersect(ray, cn, cf) && cn <= closest_t)
                        hitChildren.push_back({child, cn});
                }

                // Sort children by distance (front-to-back)
                std::sort(hitChildren.begin(), hitChildren.end(),
                    [](const ChildHit& a, const ChildHit& b) {
                        return a.tnear < b.tnear;
                    });

                // Push them in reverse order so the closest child is processed next
                for (auto it = hitChildren.rbegin(); it != hitChildren.rend(); ++it)
                    stack.push(it->node);
            }
        }
    } else {
       
        /* Brute force search through all triangles */
        for (auto idx : m_triangles) {
          uint32_t local_idx = idx;
          auto mesh = getMesh(findMesh(local_idx));
          float u, v, t;
          if (mesh->rayIntersect(idx, ray, u, v, t)) {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
            if (shadowRay)
              return true;
            hitmesh = mesh;
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            f = idx;
            foundIntersection = true;
          }
        }
    }

    if (foundIntersection) {

        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;
        const MatrixXf &V  = hitmesh->getVertexPositions();
        const MatrixXf &N  = hitmesh->getVertexNormals();
        const MatrixXf &UV = hitmesh->getVertexTexCoords();
        const MatrixXu &F  = hitmesh->getIndices();
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);
        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) + bary.y() * UV.col(idx1) + bary.z() * UV.col(idx2);
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());
        if (N.size() > 0) {
            its.shFrame = Frame(
                (bary.x() * N.col(idx0) + bary.y() * N.col(idx1) + bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }

        its.bsdf = hitmesh->getBSDF();
        if(hitmesh->isEmitter()) {
            its.emitter=hitmesh->getEmitter();
        }
    }

    return foundIntersection;
}

std::string Accel::toString() const {
    std::string meshes;
    for (size_t i=0; i<m_meshes.size(); ++i) {
        meshes += std::string("  ") + indent(m_meshes[i]->toString(), 2);
        if (i + 1 < m_meshes.size())
            meshes += ",";
        meshes += "\n";
    }
    return tfm::format(
        "Octree accel[\n"
        "  meshes = {\n"
        "  %s  }\n"
        "]",
        indent(meshes, 2)
    );
}


NORI_NAMESPACE_END