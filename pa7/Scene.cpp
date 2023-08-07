//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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
    Vector3f L_dir;
    Vector3f L_indir;
    Intersection obj_inter = intersect(ray);
    if (!obj_inter.happened) 
    {
        return L_dir;
    }
    if (obj_inter.m->hasEmission()) 
    {
        return obj_inter.m->getEmission();
    }
    Vector3f p = obj_inter.coords;
    Material* m = obj_inter.m;
    Vector3f N = obj_inter.normal.normalized();
    Vector3f w0 = ray.direction;

    Intersection light_inter;
    float pdf_light = 1.0;
    sampleLight(light_inter, pdf_light);
    Vector3f x = light_inter.coords;
    Vector3f ws = (x - p).normalized();
    Vector3f NN = light_inter.normal.normalized();
    Vector3f emit = light_inter.emit;
    float d = (x - p).norm();

    Ray Obj2Light(p, ws);
    float d2 = intersect(Obj2Light).distance;
    if (d2 - d > -0.001)
    {
        Vector3f eval = m->eval(w0, ws, N);
        float cos_theta = dotProduct(N, ws);
        float cos_theta_x = dotProduct(NN, -ws);
        L_dir = emit * eval * cos_theta * cos_theta_x / std::pow(d, 2) / pdf_light;
    }

    float P_RR = get_random_float();
    if (P_RR < RussianRoulette) 
    {
        Vector3f wi = m->sample(w0, N).normalized();
        Ray r(p, wi);
        Intersection inter = intersect(r);
        if (inter.happened && !inter.m->hasEmission())
        {
            Vector3f eval = m->eval(w0, wi, N);
            float pdf = m->pdf(w0, wi, N);
            float cos_theta = dotProduct(wi, N);
            L_indir = castRay(r, depth + 1) * eval * cos_theta / pdf / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}