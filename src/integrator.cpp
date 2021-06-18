#ifndef NO_OMP
#include <omp.h>
#endif
#include "progressbar.hpp"
#include "integrator.hpp"
#include "constant.hpp"
#include "light.hpp"
#include "utils.hpp"
#include "config.hpp"
#include "brdf.hpp"



extern Config conf;


/**
 * Integrator class
 */

Integrator::Integrator(Scene* scn, Camera* cam)
    : scene(scn)
    , camera(cam)
{
}


/**
 * PathTracingIntegrator class
 */

PathTracingIntegrator::PathTracingIntegrator(Scene* scene, Camera* camera)
    : Integrator(scene, camera)
{  
}

void PathTracingIntegrator::render()
{
    int dx, dy;
    int res_x = camera->getFilm().resolution.x(), res_y = camera->getFilm().resolution.y();

    /* Initialize a progress bar */
    progressbar progress_bar(res_x * res_y);

    /* Samples for each axis */
    int n = sqrt(conf.spp);

#ifndef NO_OMP
    #pragma omp parallel for private(dy)
#endif
    for (dx = 0; dx < res_x; ++dx)
    {
        for (dy = 0; dy < res_y; ++dy)
        {
            Eigen::Vector3f L(0, 0, 0);

            /* Sample multiple times for a pixel using jitter */
            for (int s = 0; s <= n - 1; ++s)
            {
                for (int t = 0; t <= n - 1; ++t)
                {
                    /* Generate and jitter a ray from the camera to a pixel */
                    float u =  (s + mathutils::unif(0, 1)[0]) / n;
                    float v =  (t + mathutils::unif(0, 1)[0]) / n;
                    Ray ray = camera->generateRay((float)dx + u, (float)dy + v);
                    /* Compute radiance along the sampled ray */
                    L += radiance(ray);
                }
            }
            camera->setPixel(dx, dy, L / (float)conf.spp);

#ifndef NO_OMP
            #pragma omp critical
#endif
            progress_bar.update();
        }
    }
}

Eigen::Vector3f PathTracingIntegrator::directLighting(Interaction& isect)
{
    Eigen::Vector3f L(0, 0, 0);

    Light* l = scene->getLight();

    /** Sample light source **/
    if (conf.integral_method == "mis" || conf.integral_method == "lso")
    {
        /* Sample a point from the area light and find its PDF w.r.t area */
        float light_pdf;
        Eigen::Vector3f light_pos = l->sample(isect, &light_pdf);

        /* Test visibility */
        Ray light_ray = Ray(light_pos, -isect.wi, DELTA, (light_pos - isect.entry_point).norm() - DELTA);
        bool visibility = !scene->isShadowed(light_ray);
        if (visibility)
        {
            /* Included angle between incoming light and the surface normal */
            float cos_theta_i = isect.wi.dot(isect.normal);
            /* Radiance emitted from the sampled light */
            Eigen::Vector3f Ld = l->emission(light_pos, -isect.wi);
            /* Find the BRDF and get its PDF */
            BRDF* brdf = (BRDF*)isect.material;
            /* Get the BRDF at the interaction point */
            Eigen::Vector3f f = brdf->eval(isect);

            if (abs(light_pdf) > EPSILON && abs(cos_theta_i) > EPSILON)
            {
                /* Compute the light contribution */
                Eigen::Vector3f L_l = f.cwiseProduct(Ld) * abs(cos_theta_i) / light_pdf;
                if (conf.integral_method == "mis")
                {
                    /* Compute the PDF of sampling BRDF */
                    float brdf_pdf = brdf->samplePdf(isect);
                    float w = mathutils::power_heuristic(1, light_pdf, 1, brdf_pdf);
                    L += w * L_l;
                }
                else
                {
                    L = L_l;
                }
            }
        }
    }

    /** Sample BRDF **/
    if (conf.integral_method == "mis" || conf.integral_method == "bso")
    {
        BRDF* brdf = (BRDF*)isect.material;
        /* Sample BRDF to get a incoming light ray */
        float brdf_pdf = brdf->sample(isect);
        Eigen::Vector3f f = brdf->eval(isect);
        /* Check whether the incoming light ray hits the light source */
        Interaction light_it;
        Ray light_ray = Ray(isect.entry_point, isect.wi, DELTA);
        if (abs(brdf_pdf) > EPSILON && l->rayIntersection(light_it, light_ray))
        {
            /* Check sheltering of the light ray */
            bool visibility = !scene->isShadowed(light_ray);
            if (visibility)
            {
                /* Included angle between incoming light and the surface normal */
                float cos_theta_i = isect.wi.dot(isect.normal);
                /* Radiance of the incoming light */
                Eigen::Vector3f Ld = l->emission(light_it.entry_point, -isect.wi);
                /* Compute the light contribution */
                Eigen::Vector3f L_b = f.cwiseProduct(Ld) * abs(cos_theta_i) / brdf_pdf;
                if (conf.integral_method == "mis")
                {
                    /* Compute PDF of the incoming light w.r.t solid angle */
                    float light_pdf = 0.0f;
                    if (!brdf->isDelta())
                    {
                        light_pdf = l->samplePdf(isect, light_it.entry_point);
                    }
                    float w = mathutils::power_heuristic(1, brdf_pdf, 1, light_pdf);
                    L += w * L_b;
                }
                else
                {
                    L = L_b;
                }
            }
        }
    }
    
    return L;
}

Eigen::Vector3f PathTracingIntegrator::radiance(Ray ray)
{
    Eigen::Vector3f L(0, 0, 0);
    Eigen::Vector3f beta(1.0f, 1.0f, 1.0f);

    Light* light_src = scene->getLight();
    
    bool delta_bounce = false;
    for (int bounces = 0; bounces < conf.max_depth; ++bounces)
    {
        Interaction isect;
        bool found_intersection = scene->intersection(ray, isect);
        if (!found_intersection)
        {
            break;
        }

        /* Specify outcoming direction */
        isect.wo = -ray.direction;

        if ((bounces == 0 || delta_bounce) && isect.type == Interaction::Type::LIGHT)
        {
            /* Handle light sources */
            L += beta.cwiseProduct(light_src->emission(isect.entry_point, isect.wo));
        }

        /* Ignore reflected rays hitting the light source */
        if (isect.type == Interaction::Type::LIGHT)
        {
            continue;
        }

        /* Perform direct lighting */
        L += beta.cwiseProduct(directLighting(isect));

        /* Sample BRDF to get the next path */
        BRDF* brdf = (BRDF*)isect.material;
        float brdf_pdf = brdf->sample(isect);
        if (abs(brdf_pdf) < EPSILON)
        {
            break;
        }
        delta_bounce = brdf->isDelta();
        beta = beta.cwiseProduct(brdf->eval(isect)) * abs(isect.wi.dot(isect.normal)) / brdf_pdf;
        ray = Ray(isect.entry_point, isect.wi, DELTA);
    }

    return L;
}
