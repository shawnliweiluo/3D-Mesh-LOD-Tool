#ifndef WINGED_EDGE_H
#define WINGED_EDGE_H

#include <iostream>
#include <string>
#include <cstdint>
#include <memory>
#include <utility>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <unordered_map> 
#include <unordered_set> 
#include <vector>
#include <nanogui/glutil.h>
#include <math.h>
#include <random>

using namespace std;
using namespace nanogui;

struct W_edge;
struct Vertex;
struct Face;

// TODO: use weak_ptr instead of shared_ptr
struct W_edge
{
    shared_ptr<Vertex> start, end;
    shared_ptr<Face> left, right;
    shared_ptr<W_edge> left_prev, left_next;
    shared_ptr<W_edge> right_prev, right_next;
};

struct Vertex
{
    float x, y, z;
    shared_ptr<W_edge> edge;
    int index;
    Eigen::Matrix4f Q;     // error quadric
};

struct Face
{
    shared_ptr<W_edge> edge;
    Eigen::Matrix4f Kp;    // fundamental error quadric
};

typedef pair<int, int> EdgeVertexIndices;

struct EdgeVertexIndices_Hash : public std::unary_function<EdgeVertexIndices, std::size_t>
{
    std::size_t operator()(const EdgeVertexIndices &key) const
    {
        return get<0>(key) + get<1>(key) * 3 +  get<0>(key) * get<1>(key) * 7;
    }
};

struct EdgeVertexIndices_Equal : public std::binary_function<EdgeVertexIndices, EdgeVertexIndices, bool>
{
    bool operator()(const EdgeVertexIndices &key1, const EdgeVertexIndices &key2) const
    {
        return get<0>(key1) == get<0>(key2) && get<1>(key1) == get<1>(key2);
    }
};

typedef unordered_map<EdgeVertexIndices, shared_ptr<W_edge>, EdgeVertexIndices_Hash, EdgeVertexIndices_Equal> EdgesHashMap;
typedef unordered_set<EdgeVertexIndices, EdgeVertexIndices_Hash, EdgeVertexIndices_Equal> EdgesHashSet;
typedef unordered_map<EdgeVertexIndices, int, EdgeVertexIndices_Hash, EdgeVertexIndices_Equal> EdgeIndicesToIndexMap;

struct Vertex_Hash : public std::unary_function<Vertex, std::size_t>
{
    std::double_t operator()(const Vertex &key) const
    {
        return key.x + key.y * 7 +  key.z * 5 + key.x * key.y * key.z * 3;
    }
};

struct Vertex_Equal : public std::binary_function<Vertex, Vertex, bool>
{
    bool operator()(const Vertex &key1, const Vertex &key2) const
    {
        return key1.x == key2.x && key1.y == key2.y && key1.z == key2.z;
    }
};

typedef unordered_map<Vertex, int, Vertex_Hash, Vertex_Equal> VertexToIndexMap;

enum ShadingOption
{
    FLAT_SHADING,
    SMOOTH_SHADING
};

enum SubdivideScheme
{
    LOOP,
    BUTTERFLY
};

class WingedEdgeMesh
{
public:
    WingedEdgeMesh();
    ~WingedEdgeMesh();
    void loadObj(const string& inputFile);
    void saveObj(const string& outputFile);
    const vector<shared_ptr<Vertex>>& getVertices();
    const vector<shared_ptr<Face>>& getFaces();
    const EdgesHashMap& getEdges();
    void getMeshData(const ShadingOption& shadingOption, MatrixXf &outPositions, MatrixXf &outNormals, MatrixXf &outColors);
    void subDivide(const SubdivideScheme &scheme, const int &numLevels);
    void subDivideLoop();
    void subDivideButterfly();
    void decimate(const int &k, const int &numEdges);

private:
    EdgesHashMap mEdges;
    vector<shared_ptr<Vertex>> mVertices;
    vector<shared_ptr<Face>> mFaces;

    const int REGLUAR_VALENCE = 6;

    bool loadFromObjToMats(const string &inputFile, Eigen::MatrixXf &verticesMat, Eigen::MatrixXi& facesMat);
    void updateFromMats(const Eigen::MatrixXf &verticesMat, const Eigen::MatrixXi& facesMat);
    vector<shared_ptr<Vertex>> getNeighbourVertices(shared_ptr<Vertex> vertex);
    vector<shared_ptr<Face>> getNeighbourFaces(shared_ptr<Vertex> vertex);
    void updateFaceKp();
    void updateVertexQ();
    void getMultipleChoiceBestEdge(const int &k, shared_ptr<W_edge> &bestEdge, Vector4f &newV);
    bool vectorContains(const vector<int> vec, int num);
    void clear();
    vector<string> split(const string &str, const string &delim);
};

#endif