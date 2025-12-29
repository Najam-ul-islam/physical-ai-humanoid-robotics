---
sidebar_label: ROS 2 Fundamentals
title: ROS 2 Fundamentals
description: Introduction to Robot Operating System 2 concepts and architecture
---

# ROS 2 Fundamentals

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

## Key Concepts

### Nodes
In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Each node is designed to perform a specific task and can communicate with other nodes through topics, services, and actions.

### Topics and Messages
Topics allow nodes to exchange data in a publish-subscribe pattern. Messages are the data packets sent between nodes via topics. Each message has a specific type and structure defined in ROS message definition files.

### Services
Services provide a request-response communication pattern between nodes. A client node sends a request to a service server, which processes the request and returns a response.

### Actions
Actions are used for long-running tasks that require feedback and the ability to cancel the operation. They extend the service concept with additional features for monitoring progress.

## ROS 2 Architecture

ROS 2 uses a DDS (Data Distribution Service) implementation for communication between nodes. This provides a middleware that handles message passing between nodes, even if they are running on different machines or written in different programming languages.

## Common Tools

- **ros2 run**: Execute a node from a package
- **ros2 topic**: Command-line tools for working with topics
- **ros2 service**: Command-line tools for working with services
- **rqt**: Graphical user interface tools
- **rviz**: 3D visualization tool for robot sensors and state
