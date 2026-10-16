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

PathNode PathFind(vec3 pos, vec3 playerPos) {
    int counter = 0;
    int counterMax = 100; //set max evaluation steps
    int radialSpokes = 4; //number of radial offset evaluations per step
    float radialDist = 0.1777f; //set orthogonal offset distance 'pon terrain hit; should be ∼1/2 your minimum desired apertures
    float cellSize = 0.111f; //must be smaller than radialDist
    float heuristicWeight = 2.0f; //set target weight (typically between 1-2); 1 is optimal, higher results in less optimal paths but significantly fewer evaluations

    std::priority_queue<PathNode, std::vector<PathNode>, CompareNode> openSet;
    std::unordered_set<ivec3, Vec3Hash> closedSet;
    PathNode originNode;
    originNode.pos = pos;
    ivec3 originKey = QuantizePos(pos, cellSize);
    closedSet.insert(originKey);
    originNode.cost = heuristicWeight*length(pos - playerPos); //euclidean distance heuristic 
    openSet.push(originNode);

    while(openSet.size() > 0 && counter < counterMax) {
        counter++;
        //find lowest cost node
        PathNode tempNode = openSet.top();
        openSet.pop();
        //find nearest geometry
        float sphereTrace = SDF(tempNode.pos); //call your distance estimate here
        if(sphereTrace < 0.001f) continue;
        //return if clear line of sight
        float dist2Player = length(playerPos - tempNode.pos);
        if(dist2Player < sphereTrace) {
            tempNode.checkPoints.push_back(playerPos);
            return tempNode;
        }
        //step safe distance towards target
        vec3 newPos = tempNode.pos + sphereTrace*normalize(playerPos - tempNode.pos);
        //store trajectory and distance
        tempNode.pos = newPos;
        tempNode.checkPoints.push_back(newPos);
        tempNode.distTravelled += sphereTrace;
        //check if zone already trammelled 'pon
        ivec3 parentKey = QuantizePos(newPos, cellSize);
        if (closedSet.insert(parentKey).second) {
            tempNode.cost = tempNode.distTravelled + (heuristicWeight*(length(playerPos - newPos))); //euclidean distance heuristic
            openSet.push(tempNode);
        }

        //get surface normal and orthonormal basis
        vec3 newNormal = Normal(newPos, sphereTrace); //call your normal estimate here
        vec3 upVec = (abs(newNormal.y) < 0.99f) ? vec3(0.0f, 1.0f, 0.0f) : vec3(1.0f, 0.0f, 0.0f);
        vec3 t1 = normalize(cross(newNormal, upVec));
        vec3 t2 = cross(newNormal, t1);
        //generate safe offset distance to prevent tunnelling
        float skin = 0.1333f; //must be smaller than radialDist
        vec3 basePos = newPos + skin*newNormal;
        float baseClearance = SDF(basePos); //call your distance estimate here
        if (baseClearance < 1e-3f) continue;
        float offset = min(radialDist, baseClearance - 1e-3f);

        //evaluate offset positions
        for(int i = 1; i <= radialSpokes; i++) {
            //generate orthogonal vector
            float angle = i * (360.0f/radialSpokes);
            angle = radians(angle);
            vec3 tangentVec = cos(angle) * t1 + sin(angle) * t2;
            //offset position
            vec3 radialPos = basePos + (offset*tangentVec);
            //geometry distance
            float sphereTrace3 = SDF(radialPos); //call your distance estimate here
            //return if clear path
            //step
            if(sphereTrace3 > 0.001f) { //check if offset intersects or tunnels
                //step safe distance towards target
                vec3 stepPos = radialPos + sphereTrace3*normalize(playerPos - radialPos);                
                //check if zone already trammelled 'pon
                ivec3 childKey = QuantizePos(stepPos, cellSize);
                if(closedSet.find(childKey) != closedSet.end()) continue;
                closedSet.insert(childKey);
                //create node and store trajectory, distance, cost
                //ibid. ancestor node's 
                PathNode radialNode;
                float dist3Player = length(playerPos - radialPos);
                if(dist3Player < sphereTrace3) {
                    radialNode = tempNode; 
                    radialNode.checkPoints.push_back(basePos);
                    radialNode.checkPoints.push_back(radialPos);
                    radialNode.checkPoints.push_back(playerPos);
                    return radialNode;
                }
                radialNode.pos = stepPos;
                radialNode.checkPoints = tempNode.checkPoints;
                radialNode.checkPoints.push_back(radialPos);
                radialNode.checkPoints.push_back(stepPos);
                radialNode.distTravelled = tempNode.distTravelled + offset + sphereTrace3;
                radialNode.cost = radialNode.distTravelled + (heuristicWeight*(length(playerPos - stepPos))); //euclidean distance heuristic 
              
                openSet.push(radialNode);
            }
        }
    }
    //no path found
    PathNode failureNode;
    failureNode.distTravelled = -777.0f;
    return failureNode;
}
