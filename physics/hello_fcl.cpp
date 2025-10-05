#include <fcl/fcl.h>
#include <iostream>

int main() {
    auto box1 = std::make_shared<fcl::Boxd>(1.0, 1.0, 1.0);
    auto box2 = std::make_shared<fcl::Boxd>(1.0, 1.0, 1.0);

    fcl::Transform3d tf1 = fcl::Transform3d::Identity();
    fcl::Transform3d tf2 = fcl::Transform3d::Identity();
    tf2.translation() << 0.5, 0.0, 0.0; // overlap of 0.5 along x

    // collsion check
    fcl::CollisionObjectd obj1(box1, tf1);
    fcl::CollisionObjectd obj2(box2, tf2);
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    fcl::collide(&obj1, &obj2, request, result);
    std::cout << "is Collision ? " << result.isCollision() << std::endl;

    // distance check
    tf2.translation() << 3.0, 0.0, 0.0;
    obj2.setTransform(tf2);
    fcl::DistanceRequestd distance_request;
    fcl::DistanceResultd distance_result;
    fcl::distance(&obj1, &obj2, distance_request, distance_result);
    std::cout << "Distance: " << distance_result.min_distance << std::endl;

    // continuous collision check
    fcl::Transform3d tf_goal_1 = fcl::Transform3d::Identity();
    fcl::Transform3d tf_goal_2 = fcl::Transform3d::Identity();
    fcl::ContinuousCollisionRequestd cc_request;
    fcl::ContinuousCollisionResultd cr_result;
    fcl::continuousCollide(
        &obj1, tf_goal_1, &obj2, tf_goal_2, cc_request, cr_result);
    std::cout << "is Collision ?" << cr_result.is_collide << std::endl;
    return 0;
}
