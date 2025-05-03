//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "global.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }

    // RR
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);
    if (!inter.happened)
    {
        return {};
    }

    return shade(inter, -ray.direction);

}


Vector3f Scene::shade(const Intersection &_intersection, const Vector3f &wo) const
{

    if (_intersection.obj->hasEmit())
    {
        return _intersection.emit;
    }

    const Vector3f &p = _intersection.coords;
    const Vector3f &N = _intersection.normal;

    Vector3f L_dir, L_indir;
    
    float light_pdf = 0.0;
    Intersection light_iter;

    sampleLight(light_iter, light_pdf);

    Vector3f &x = light_iter.coords;
    Vector3f &NN = light_iter.normal;
    Vector3f ws = p - x;
    Vector3f wsdir = ws.normalized();
    float ws_norm = ws.norm();

    if (Intersection iter = intersect(Ray{p, -wsdir}); iter.distance - ws_norm > -0.01)
    {
        float cos_theta = dotProduct(N, -wsdir);
        float cos_theta_x = dotProduct(NN, wsdir);
        const Vector3f fr = _intersection.m->eval(wo, -wsdir, N);
        L_dir = light_iter.emit * fr * cos_theta * cos_theta_x / (ws_norm * ws_norm) / light_pdf;
    }

    if (get_random_float() < RussianRoulette)
    {
        const Vector3f wi = normalize(_intersection.m->sample(wo, N));
        const float pdf = _intersection.m->pdf(wo, wi, N);
        const Vector3f fr = _intersection.m->eval(wo, wi, N);

        if (Intersection iter = intersect(Ray{_intersection.coords, wi}); iter.happened && !iter.obj->hasEmit() && pdf > 0.01)
        {
            L_indir = shade(iter, -wi) * fr * dotProduct(wi, N) / pdf / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}