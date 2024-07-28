---
layout: post
title: Robot Singularity
subtitle: A quick overview on techniques to handle singularities
author: "fcjscott"
header-style: text
lang: en
mathjax: true
tags:
  - Robotics
  - Kinematics
  - Manipulator
  - Intermediate
---

# Overview
Singularity is a big topic in robotics, particularly for manipulators and legged robots. Simply speaking, singularity occurs when the manipulator cannot follow a task space command because there are infinite number of joint space configuration at singularity pose. Mathematically, it means the Jacobian matrix, $J(\theta)$, of the manipulator is rank deficient, which causes the robot to lose one or more degrees of freedom. We can think the Jacobian matrix as a mapping relationship between the joint space and task space. When the Jacobian matrix of a manipulator is rank deficient, it indicates that a linear combination of multiple joints will produce the same pose in task space. This results into the controller not be able to follow the control signals send to it, which consequently causes execution failures.

# How to overcome singularity?
Throughout my journey as a robotics engineer, especially when working on motion planning problems, I often find singularity doesn't occur quite often, but when it occurs, it is quite problematic. This is particularly true when there are strict task space constraints, such as following waypoints in end-effector's space with interval of 1 mm. Also, different problem settings will require different approaches to deal with singularity, and I will cover those approaches that I hope you will find them insightful.

## Additional DoF
Adding degrees of freedom is one of the most effective ways to address singularity since it maintains the original functional requirements for performing tasks. However, in practice, this solution is often considered luxurious due to the additional costs and complications involved. For instance, you can add a DoF by using a moving platform or a higher-DoF robot manipulator. However, scalability, both product-wise and financially, becomes a concern. In addition, extra DoFs can reduce the robot's rigidity and require more effort in system calibration. Both technical and non-technical factors must be considered, and the decision often varies depending on the application.

## Direct Avoidance - Sampling Based
Direct avoidance can complement adding degrees of freedom. If your task has redundant degrees of freedom, that means it can be utilized to avoid the singularity. This sampling based approach can be utilized to generate samples in planning space, then you solve for your inverse kinematics to get our joint space solutions. Among these solutions, you identify which solutions are closer to singularity and which are farther, thus defining a threshold to reject or accept samples. In fact, the "closeness" to singularity has its official name called [**_manipulability_**][1], is defined as following: 

<div align="center">
$ m(\theta) = \det(\sqrt{J(\theta)J^T(\theta)})$
</div>

Where $\theta$ represents the joint space angles, and $m(\theta)$ represents manipulability index.

After we reject the samples whose $m$ is larger than the threshold, we can use numerous classical path-finding algorithms - again, depends on application, to find the shortest path from the start state to the end state. It is worth to mention that this approach can be integrated into traditional sampling-based planner, such as _RRT_, _PRM_ as well. It really depends on what kind of use cases to make full use of this method.

## Direct Avoidance - SR-Inversion of Jacobian
This is a more advanced approach to singularity avoidance. From an application standpoint, if your task requires constraining all 6 degrees of freedom in cartesian space, this method is ideal becuase it enforces to maintain the end-effector on cartesian space. Only when approaching a singularity, the cartesian pose will then be slightly deviate. This smart approach was first introduced in [this paper][2], and I recommend you to read through that paper to understand more thorouhly. Here, I will provide a high-level overview.

To summarize what this paper does, we first perform a singular value decomposition of the Jacobian matrix:

<div align="center">
$ J(\theta) = U_j\Sigma_jV_j^T $
</div>

Where $\Sigma_j^T$ is a diagnal singular value matrix. To compute the joint state from cartesian pose, we can write

<div align="center">
$\delta q_{tip} = V_{j}^{-1}\Sigma_{j}^{-1}U_j\delta p $
</div>

Note that if the $J$ is a singular matrix, it is impossible to direclty solve for $\theta$ because $J$ is non-inversible. Consequently, the above equation won't hold if the robot is in singular configuration. To ensure the Jacobian inversible at all the time, the SR-inversion approach has been introduced. We can rewrite the SR-inversion of the Jacobian matrix as

<div align="center">
$J^* = J^{T}(JJ^T+kI)^{-1}$
</div>

In this equation, we simply add a samll perturbation of $k$ on each entry to make the inversion of Jacobian to be always non-singular, simply because $kI$ cannot be rank deficient.

By doing this, we can always solves for joint space solutions from a cartesion pose, regardless of whether the configuration is singular or not. However, we should be aware that since the perturbation $k$ is added on all entries and all the times, it is possible that the robot can be less accurate in cartesian space. Implementer of this algorithm can either enforce SR-inversion as needed, or smartly pick $k$ values to mitigate this issue.

[1]: https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-4-manipulability/#department
[2]: https://asmedigitalcollection.asme.org/dynamicsystems/article-abstract/108/3/163/425826/Inverse-Kinematic-Solutions-With-Singularity?redirectedFrom=fulltext