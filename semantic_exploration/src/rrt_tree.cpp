/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <semantic_exploration/rrt_tree.h>

rrtNBV::Node::Node()
{
    parent_ = NULL;
    distance_ = DBL_MAX;
    gain_ = 0.0;
}

rrtNBV::Node::~Node()
{
    for (typename std::vector<Node *>::iterator it = children_.begin(); it != children_.end(); it++)
    {
        delete (*it);
        (*it) = NULL;
    }
}

rrtNBV::TreeBase::TreeBase()
{
    bestGain_ = params_.zero_gain_;
    bestNode_ = NULL;
    bestObjectGain_ = params_.zero_gain_;
    bestObjectNode_ = NULL;
    counter_ = 0;
    rootNode_ = NULL;
}

rrtNBV::TreeBase::TreeBase(OctomapGeneratorBase *octomap_generator_)
{
    manager_ = octomap_generator_;
    bestGain_ = params_.zero_gain_;
    bestNode_ = NULL;
    bestObjectGain_ = params_.zero_gain_;
    bestObjectNode_ = NULL;
    counter_ = 0;
    rootNode_ = NULL;
}

rrtNBV::TreeBase::~TreeBase()
{
}

void rrtNBV::TreeBase::setParams(Params params)
{
    params_ = params;
}

int rrtNBV::TreeBase::getCounter()
{
    return counter_;
}

bool rrtNBV::TreeBase::gainFound()
{
    return bestGain_ > params_.zero_gain_;
}

Eigen::Vector4d rrtNBV::TreeBase::getRootNode()

{
    return rootNode_->state_;
}
