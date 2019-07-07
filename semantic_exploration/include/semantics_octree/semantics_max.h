/**
* \author Xuan Zhang
* \data Mai-July 2018
*/
#ifndef SEMANTICS_MAX_H
#define SEMANTICS_MAX_H

#include <octomap/ColorOcTree.h>

namespace octomap
{
/// Structure contains semantic colors and their confidences
struct SemanticsMax
{
    ColorOcTreeNode::Color semantic_color;  ///<Semantic color
    float confidence;
    uint  numVisits;
    SemanticsMax() : semantic_color(), confidence(0.),numVisits(.0)
    {
    }

    bool operator==(const SemanticsMax& rhs) const
    {
        return semantic_color == rhs.semantic_color && confidence == rhs.confidence;
    }

    bool operator!=(const SemanticsMax& rhs) const
    {
        return !(*this == rhs);
    }

    ColorOcTreeNode::Color getSemanticColor() const
    {
        //std::cout << "semantic_color" <<  float(semantic_color.r)  << "  " << float(semantic_color.g) << "  " <<  float(semantic_color.b) << std::endl << std::flush ; 
        return semantic_color;
    }

    bool isSemanticsSet() const
    {
        if (semantic_color != ColorOcTreeNode::Color(255, 255, 255))
            return true;
        return false;
    }

    uint incrementNumVisits()
    {
       return ++numVisits;
    }

    uint getNumVisits()
    {
        return numVisits;
    }

    /// Perform max fusion
    static SemanticsMax semanticFusion(const SemanticsMax s1, const SemanticsMax s2)
    {
        SemanticsMax ret;
        // If the same color, update the confidence to the average
        if (s1.semantic_color == s2.semantic_color)
        {
            ret.semantic_color = s1.semantic_color;
            ret.confidence = (s1.confidence + s2.confidence) / 2.;
        }
        // If color is different, keep the larger one and drop a little for the disagreement
        else
        {
            ret = s1.confidence > s2.confidence ? s1 : s2;
            ret.confidence *= 0.9;
        }
        return ret;
    }
};

std::ostream& operator<<(std::ostream& out, SemanticsMax const& s);
}  // namespace octomap
#endif  //SEMANTICS_MAX_H
