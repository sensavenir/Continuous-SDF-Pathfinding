#include <unordered_set>
#include <queue>
#include <vector>

#include "glm/glm.hpp" 
#include "glm/ext.hpp"
using namespace glm;

struct Vec3Hash {
    size_t operator()(const ivec3& v) const {
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 1) ^ (std::hash<int>()(v.z) << 2);
    }
};

struct PathNode {
    std::vector<vec3> checkPoints = {};
    vec3 pos = vec3(0.0f);
    float cost = 0.0f;
    float distTravelled = 0.0f;
};

struct CompareNode {
    bool operator()(const PathNode& a, const PathNode& b) const {
        return a.cost > b.cost;
    }
};

ivec3 QuantizePos(vec3 pos, float cellSize) {
    return ivec3(floor(pos.x / cellSize), floor(pos.y / cellSize), floor(pos.z / cellSize));
}

PathNode PathFind(vec3 pos, vec3 playerPos, vec3 playerDir) {
    int counter = 0;
    int counterMax = 100; //set max evaluation steps
    int radialSpokes = 4; //number of radial offset evaluations per step
    float radialDist = 0.333f; //set orthogonal offset distance 'pon terrain hit; should be ~1/2 your minimum desired apertures (pause)
    float heuristicWeight = 1.0f; //set target weight (typically between 1-2); 1 is optimal, higher results in less optimal paths but significantly fewer evaluations

    std::priority_queue<PathNode, std::vector<PathNode>, CompareNode> openSet;
    std::unordered_set<ivec3, Vec3Hash> closedSet;
    PathNode originNode;
    originNode.pos = pos;
    originNode.cost = heuristicWeight*length(pos - playerPos); //euclidean distance heuristic 
    openSet.push(originNode);

    while(openSet.size() > 0 && counter < 100) {
        //find lowest cost node
        PathNode tempNode = openSet.top();
        openSet.pop();
        //find nearest geometry
        float sphereTrace = YourSDF(tempNode.pos);
        //step safe distance towards target
        vec3 newPos = tempNode.pos + sphereTrace*normalize(playerPos - tempNode.pos);
        //check if zone already trammelled 'pon
        ivec3 key = QuantizePos(newPos, radialDist);
        if(closedSet.find(key) != closedSet.end()) continue;
        closedSet.insert(key);
        
        //store trajectory and distance
        tempNode.checkPoints.push_back(newPos);
        tempNode.distTravelled += sphereTrace;

        //get surface normal and orthonormal basis
        vec3 newNormal = YourNormal(newPos, sphereTrace); //call your normal estimate here
        vec3 upVec = (abs(newNormal.y) < 0.99f) ? vec3(0.0f, 1.0f, 0.0f) : vec3(1.0f, 0.0f, 0.0f);
        vec3 t1 = normalize(cross(newNormal, upVec));
        vec3 t2 = cross(newNormal, t1);

        //evaluate offset positions
        for(int i = 1; i <= radialSpokes; i++) {
            //generate orthogonal vector
            float angle = i * (360.0f/radialSpokes);
            angle = radians(angle);
            vec3 tangentVec = cos(angle) * t1 + sin(angle) * t2;
            //new position
            vec3 radialPos = newPos + radialDist*tangentVec;
            //geometry distance
            sphereTrace = YourSDF(radialPos); //call your distance estimate
            //step
            if(sphereTrace > 0.001f) {
                //step safe distance towards target
                vec3 stepPos = radialPos + sphereTrace*normalize(playerPos - radialPos);
                //create node and store trajectory, distance, cost
                //ibid. ancestor node's 
                PathNode radialNode;
                radialNode.pos = stepPos;
                radialNode.checkPoints = tempNode.checkPoints;
                radialNode.checkPoints.push_back(radialPos);
                radialNode.checkPoints.push_back(stepPos);
                radialNode.distTravelled += tempNode.distTravelled + radialDist + sphereTrace;
                radialNode.cost = radialNode.distTravelled + (heuristicWeight*(length(playerPos - stepPos))); //euclidean distance heuristic
                
                //return if clear path
                float dist2Player = length(playerPos - radialPos);
                if(dist2Player < sphereTrace) {
                    return radialNode;
                }
                openSet.push(radialNode);
            }
        }
        counter++;
    }
    //no path found
    PathNode failureNode;
    return failureNode;
}
