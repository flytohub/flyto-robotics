# Contracts

This directory contains versioned JSON contracts used to keep planning,
execution, observations, and evidence separate.

The current production boundary is external-computer execution over standard
ROS 2. Historical delivery/job-runner contracts remain for regression evidence
but do not require Flyto2 software on a robot.

A ROS/Nav2 action result is an execution fact. Task completion remains a Cloud
verification decision bound to the original goal and independent evidence.
