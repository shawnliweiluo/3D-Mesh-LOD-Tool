#include "wingededge.h"

using namespace std;

WingedEdgeMesh::WingedEdgeMesh()
{

}

WingedEdgeMesh::~WingedEdgeMesh()
{
    clear();
}

bool WingedEdgeMesh::loadFromObjToMats(const string &inputFile, Eigen::MatrixXf &verticesMat, Eigen::MatrixXi& facesMat)
{
    ifstream infile(inputFile);
    string line;
    bool firstLine = true;

    int curVertexIndex = 0;
    int curFaceIndex = 0;

    while (getline(infile, line))
    {
        // first line should be in the format: # num_vertices num_faces
        if(firstLine)
        {
            if(line.empty())
            {
                continue;
            }
            if(line[0] == '#')
            {
                firstLine = false;
                vector<string> tokens = split(line, " ");
                int numVertices = stoi(tokens[1]);
                int numFaces = stoi(tokens[2]);
                verticesMat = Eigen::MatrixXf(3, numVertices);
                facesMat = Eigen::MatrixXi(3, numFaces);
            }
            else
            {
                cout << "Failed to load obj file: first line does not contain its size info." << endl;
                return false;
            }            
        }

        // ignore empty lines and comments
        if(line.empty() || line[0] == '#')
        {
            continue;
        }

        vector<string> tokens = split(line, " ");

        // vertex data
        if(line[0] == 'v')
        {
            float x = stof(tokens[1]);
            float y = stof(tokens[2]);
            float z = stof(tokens[3]);
            verticesMat.col(curVertexIndex) << x, y, z;
            curVertexIndex++;
        }

        // face data
        else if(line[0] == 'f')
        {
            // vertices in obj files are 1-indexed
            int vertex1Index = stoi(tokens[1]) - 1;
            int vertex2Index = stoi(tokens[2]) - 1;
            int vertex3Index = stoi(tokens[3]) - 1;
            facesMat.col(curFaceIndex) << vertex1Index, vertex2Index, vertex3Index;
            curFaceIndex++;
        }
    }
    infile.close();
    return true;
}

void WingedEdgeMesh::updateFromMats(const Eigen::MatrixXf &verticesMat, const Eigen::MatrixXi& facesMat)
{
    clear();
    
    for (int i = 0; i < verticesMat.cols(); i++)
    {
        shared_ptr<Vertex> vertex(new Vertex);
        vertex->x = verticesMat.col(i).x();
        vertex->y = verticesMat.col(i).y();
        vertex->z = verticesMat.col(i).z();
        vertex->index = i;
        mVertices.push_back(vertex);
    }
    
    for (int i = 0; i < facesMat.cols(); i++)
    {
        int vertex1Index = facesMat.col(i).x();
        int vertex2Index = facesMat.col(i).y();
        int vertex3Index = facesMat.col(i).z();

        shared_ptr<Vertex> vertex1 = mVertices.at(vertex1Index);
        shared_ptr<Vertex> vertex2 = mVertices.at(vertex2Index);
        shared_ptr<Vertex> vertex3 = mVertices.at(vertex3Index);
        
        // look up 3 edge pairs in counter-clockwise order
        EdgeVertexIndices indices12 = make_pair(vertex1Index, vertex2Index);
        EdgesHashMap::const_iterator itr = mEdges.find(indices12);
        if (itr == mEdges.end())
        {
            shared_ptr<W_edge> edge12(new W_edge);
            mEdges[indices12] = edge12;
        }
        EdgeVertexIndices indices21 = make_pair(vertex2Index, vertex1Index);
        itr = mEdges.find(indices21);
        if (itr == mEdges.end())
        {
            shared_ptr<W_edge> edge21(new W_edge);
            mEdges[indices21] = edge21;
        }
        EdgeVertexIndices indices23 = make_pair(vertex2Index, vertex3Index);
        itr = mEdges.find(indices23);
        if (itr == mEdges.end())
        {
            shared_ptr<W_edge> edge23(new W_edge);
            mEdges[indices23] = edge23;
        }
        EdgeVertexIndices indices32 = make_pair(vertex3Index, vertex2Index);
        itr = mEdges.find(indices32);
        if (itr == mEdges.end())
        {
            shared_ptr<W_edge> edge32(new W_edge);
            mEdges[indices32] = edge32;
        }
        EdgeVertexIndices indices31 = make_pair(vertex3Index, vertex1Index);
        itr = mEdges.find(indices31);
        if (itr == mEdges.end())
        {
            shared_ptr<W_edge> edge31(new W_edge);
            mEdges[indices31] = edge31;
        }
        EdgeVertexIndices indices13 = make_pair(vertex1Index, vertex3Index);
        itr = mEdges.find(indices13);
        if (itr == mEdges.end())
        {
            shared_ptr<W_edge> edge13(new W_edge);
            mEdges[indices13] = edge13;
        }
        shared_ptr<Face> face(new Face);
        face->edge = mEdges[indices12];
        mFaces.push_back(face);

        mVertices.at(vertex1Index)->edge = mEdges[indices12];
        mEdges[indices12]->start = vertex1;
        mEdges[indices12]->end = vertex2;
        mEdges[indices12]->left = face;
        mEdges[indices12]->left_prev = mEdges[indices32];
        mEdges[indices12]->left_next = mEdges[indices13];

        mEdges[indices21]->start = vertex2;
        mEdges[indices21]->end = vertex1;
        mEdges[indices21]->right = face;
        mEdges[indices21]->right_prev = mEdges[indices32];
        mEdges[indices21]->right_next = mEdges[indices13];

        mVertices.at(vertex2Index)->edge = mEdges[indices23];
        mEdges[indices23]->start = vertex2;
        mEdges[indices23]->end = vertex3;
        mEdges[indices23]->left = face;
        mEdges[indices23]->left_prev = mEdges[indices13];
        mEdges[indices23]->left_next = mEdges[indices21];

        mEdges[indices32]->start = vertex3;
        mEdges[indices32]->end = vertex2;
        mEdges[indices32]->right = face;
        mEdges[indices32]->right_prev = mEdges[indices13];
        mEdges[indices32]->right_next = mEdges[indices21];

        mVertices.at(vertex3Index)->edge = mEdges[indices31];
        mEdges[indices31]->start = vertex3;
        mEdges[indices31]->end = vertex1;
        mEdges[indices31]->left = face;
        mEdges[indices31]->left_prev = mEdges[indices21];
        mEdges[indices31]->left_next = mEdges[indices32];

        mEdges[indices13]->start = vertex1;
        mEdges[indices13]->end = vertex3;
        mEdges[indices13]->right = face;
        mEdges[indices13]->right_prev = mEdges[indices21];
        mEdges[indices13]->right_next = mEdges[indices32];
    }
}

void WingedEdgeMesh::loadObj(const string &inputFile)
{
    Eigen::MatrixXf verticesMat;
    Eigen::MatrixXi facesMat;
    if(!loadFromObjToMats(inputFile, verticesMat, facesMat))
    {
        return;
    }
    updateFromMats(verticesMat, facesMat);
}

void WingedEdgeMesh::saveObj(const string &outputFile)
{
    ofstream newFile(outputFile);
    
    if(newFile.is_open())   
    {
        int numFaces = mFaces.size();
        int numVertices = mVertices.size();
        
        newFile << "# " << numVertices << " " << numFaces << endl;

        VertexToIndexMap vertexToIndex;
        for(int i = 0; i < numVertices; i++)
        {
            auto v = mVertices.at(i);
            newFile << "v " << v->x << " " << v->y << " " << v->z << endl;
            vertexToIndex[*v] = i + 1; // 1-indexed
        }
        
        for(int i = 0; i < numFaces; i++)
        {
            auto f = mFaces.at(i);
            auto v1 = f->edge->start;
            auto v2 = f->edge->end;
            auto v3 = f->edge->left_prev->start;
            newFile << "f " << vertexToIndex[*v1] << " " << vertexToIndex[*v2] << " " << vertexToIndex[*v3] << endl;
        }
    }
    else 
    {
        cout << "Failed to open file for saving: " << outputFile << endl;
    }

    newFile.close();
}

void WingedEdgeMesh::getMeshData(const ShadingOption& shadingOption, MatrixXf &outPositions, MatrixXf &outNormals, MatrixXf &outColors)
{
    int numFaces = mFaces.size();
    int numVertices = numFaces * 3; // each triangle(face) outputs 3 vertices
    int numEdges = mEdges.size();
    outPositions = MatrixXf(3, numVertices * 3); // each triangle(face) also outputs 6 vertices for edges
    MatrixXf facePos = MatrixXf(3, numVertices);
    MatrixXf linePos = MatrixXf(3, numVertices * 2);
    outNormals = MatrixXf(3, numVertices);
    outColors = MatrixXf(3, numVertices);
    
    for(int i = 0; i < numVertices; i++)
    {
        // calculate new face positions
        auto currentEdge = mFaces.at(i/3)->edge;
        shared_ptr<Vertex> v;
        if(i % 3 == 0)
        {
            v = currentEdge->start;
            facePos.col(i) << v->x, v->y, v->z;
        }
        else if(i % 3 == 1)
        {
            v = currentEdge->end;
            facePos.col(i) << v->x, v->y, v->z;
        }
        else
        {
            v = currentEdge->left_prev->start;
            facePos.col(i) << v->x, v->y, v->z;
        }
        
        // calculate new vertex normals
        auto e0 = v->edge;
        auto edge = e0;
        Vector3f vertexNormal = {0, 0, 0};
        Vector3f currentFaceNormal = {0, 0, 0};
        bool currentFaceNormalRecorded = false;
        int maxLoopCount = 100;
        int curLoopCount = 0;
        do
        {
            if(curLoopCount >= maxLoopCount)
            {
                cout << "WingedEdgeMesh::getMeshData finding faces around each vertex maxLoopCount exceeded." << endl;
                break;
            }

            shared_ptr<Face> face;
            if(edge->end == v)
            {
                face = edge->right;
                edge = edge->right_next;
            }
            else
            {
                face = edge->left;
                edge = edge->left_next;
            }
            Vector3f vec1 = {face->edge->end->x - face->edge->start->x,
                            face->edge->end->y - face->edge->start->y, 
                            face->edge->end->z - face->edge->start->z};
            Vector3f vec2 = {face->edge->left_next->end->x - face->edge->left_next->start->x, 
                            face->edge->left_next->end->y - face->edge->left_next->start->y, 
                            face->edge->left_next->end->z - face->edge->left_next->start->z};
            Vector3f faceNormal = vec1.cross(vec2);
            vertexNormal += faceNormal;

            if(!currentFaceNormalRecorded)
            {
                currentFaceNormal = faceNormal.normalized();
                currentFaceNormalRecorded = true;
            }

            if(shadingOption == ShadingOption::FLAT_SHADING)
            {
                break;
            }

            curLoopCount++;
        } while (edge != e0);

        outNormals.col(i) << vertexNormal.normalized();
        // add new 3 lines(6 linePos elements) at the 3rd(last) vertex of each face
        if(i % 3 == 2)
        {
            Vector3f lineVertex1 = facePos.col(i - 2) + currentFaceNormal * 0.0005;
            Vector3f lineVertex2 = facePos.col(i - 1) + currentFaceNormal * 0.0005;
            Vector3f lineVertex3 = facePos.col(i) + currentFaceNormal * 0.0005;
            linePos.col(2*(i - 2)) << lineVertex1;
            linePos.col(2*(i - 2) + 1) << lineVertex2;
            linePos.col(2*(i - 2) + 2) << lineVertex2;
            linePos.col(2*(i - 2) + 3) << lineVertex3;
            linePos.col(2*(i - 2) + 4) << lineVertex3;
            linePos.col(2*(i - 2) + 5) << lineVertex1;
        }

        // default light purple color
        outColors.col(i) << 0.5, 0.5, 0.8;
    }

    outPositions << facePos, linePos;

    // center positions at origin and normalize 
    Vector3f center = facePos.rowwise().mean();
    outPositions.colwise() -= center;
    float maxAbsCoeff = facePos.array().abs().maxCoeff();
    if(maxAbsCoeff != 0)
    {
        outPositions /= maxAbsCoeff;
    }    
}

void WingedEdgeMesh::subDivide(const SubdivideScheme &scheme, const int& numLevels)
{
    if(mVertices.empty() || mFaces.empty() || mEdges.empty())
    {
        return;
    }

    for(int i = 0; i < numLevels; i++)
    {
        if(scheme == SubdivideScheme::LOOP)
        {
            subDivideLoop();
        }
        else
        {
            subDivideButterfly();
        }
    }
}

void WingedEdgeMesh::subDivideLoop()
{
    int numNewVertices = mEdges.size() / 2;
    Eigen::MatrixXf newVertices(3, mVertices.size() + numNewVertices);
    Eigen::MatrixXi newFaces(3, mFaces.size() * 4);

    // vertex rule
    for (int i = 0; i < mVertices.size(); i++)
    {
        auto vertex = mVertices.at(i);
        auto e0 = vertex->edge;
        auto edge = e0;
        float newX = 0, newY = 0, newZ = 0;
        vector<shared_ptr<Vertex>> neighbourVertices = getNeighbourVertices(vertex);
        int valence = neighbourVertices.size();
        float neighbourWeight = 1 / 16.0;

        if(valence != REGLUAR_VALENCE)
        {
            neighbourWeight = (1.0 / valence) * (5.0 / 8 - pow(3.0 / 8 + 0.25 * cos(2 * M_PI / valence), 2));
        }

        for (auto neighbour : neighbourVertices)
        {
            newX += neighbourWeight * neighbour->x;
            newY += neighbourWeight * neighbour->y;  
            newZ += neighbourWeight * neighbour->z;
        }
        float selfWeight = 1 - valence * neighbourWeight;
        newX += selfWeight * vertex->x;
        newY += selfWeight * vertex->y;
        newZ += selfWeight * vertex->z;
        newVertices.col(i) << newX, newY, newZ;
    }

    // edge rule
    int newVertexIndex = mVertices.size();
    EdgeIndicesToIndexMap edgeToNewVertexIndex;
    for (int i = 0; i < mFaces.size(); i++)
    {        
        auto face = mFaces.at(i);
        auto edge1 = face->edge;            // ccw
        auto edge2 = edge1->left_next;      // cw
        auto edge3 = edge2->right_next;     // cw
        int v1Index = edge1->start->index;
        int v2Index = edge1->end->index;
        int v3Index = edge2->end->index;
        vector<shared_ptr<W_edge>> faceEdges{edge1, edge2, edge3};
        vector<int> newVerticesIndices;

        for (auto edgeCur : faceEdges)
        {
            auto vertexStart = edgeCur->start;
            auto vertexEnd = edgeCur->end;
            auto vertexLeft = edgeCur->left_next->end;
            auto vertexRight = edgeCur->right_next->end;
            int vertexStartIndex = vertexStart->index;
            int vertexEndIndex = vertexEnd->index;
            float newX = 3.0 / 8 * vertexStart->x + 3.0 / 8 * vertexEnd->x + 1.0 / 8 * vertexLeft->x + 1.0 / 8 * vertexRight->x;
            float newY = 3.0 / 8 * vertexStart->y + 3.0 / 8 * vertexEnd->y + 1.0 / 8 * vertexLeft->y + 1.0 / 8 * vertexRight->y;
            float newZ = 3.0 / 8 * vertexStart->z + 3.0 / 8 * vertexEnd->z + 1.0 / 8 * vertexLeft->z + 1.0 / 8 * vertexRight->z;
            EdgeVertexIndices indices = make_pair(vertexStartIndex, vertexEndIndex);
            EdgeIndicesToIndexMap::const_iterator itr = edgeToNewVertexIndex.find(indices);
            if (itr == edgeToNewVertexIndex.end())
            {
                newVerticesIndices.push_back(newVertexIndex);
                EdgeVertexIndices reverseIndices = make_pair(vertexEndIndex, vertexStartIndex);
                edgeToNewVertexIndex[indices] = newVertexIndex;
                edgeToNewVertexIndex[reverseIndices] = newVertexIndex;            
                newVertices.col(newVertexIndex) << newX, newY, newZ;
                newVertexIndex++;
            }
            else
            {
                newVerticesIndices.push_back(edgeToNewVertexIndex[indices]);
            }
        }

        int newVertex1Index = newVerticesIndices.at(0);
        int newVertex2Index = newVerticesIndices.at(1);
        int newVertex3Index = newVerticesIndices.at(2);
 
        newFaces.col(4 * i) << v2Index, newVertex3Index, newVertex1Index;
        newFaces.col(4 * i + 1) << newVertex1Index, newVertex3Index, newVertex2Index;
        newFaces.col(4 * i + 2) << newVertex1Index, newVertex2Index, v1Index;
        newFaces.col(4 * i + 3) << newVertex3Index, v3Index, newVertex2Index;
    }

    // update
    updateFromMats(newVertices, newFaces);
}

void WingedEdgeMesh::subDivideButterfly()
{
    int numNewVertices = mEdges.size() / 2;
    Eigen::MatrixXf newVertices(3, mVertices.size() + numNewVertices);
    Eigen::MatrixXi newFaces(3, mFaces.size() * 4);

    // copy old vertices
    for (int i = 0; i < mVertices.size(); i++)
    {
        auto vertex = mVertices.at(i);        
        newVertices.col(i) << vertex->x, vertex->y, vertex->z;
    }

    // edge rule
    int newVertexIndex = mVertices.size();
    int curFaceIndex = 0;
    EdgeIndicesToIndexMap edgeToNewVertexIndex;
    for (int i = 0; i < mFaces.size(); i++)
    {       
        auto face = mFaces.at(i);
        auto edge1 = face->edge;            // ccw
        auto edge2 = edge1->left_next;      // cw
        auto edge3 = edge2->right_next;     // cw
        int v1Index = edge1->start->index;
        int v2Index = edge1->end->index;
        int v3Index = edge2->end->index;
        vector<shared_ptr<W_edge>> faceEdges{edge1, edge2, edge3};
        vector<int> newVerticesIndices;

        for (int j = 0; j < faceEdges.size(); j++)
        {
            auto edgeCur = faceEdges.at(j);
            auto vertexMidLeft = j == 0 ? edgeCur->end : edgeCur->start;
            auto vertexMidRight = j == 0 ? edgeCur->start : edgeCur->end;
            auto vertexTopMid = j == 0 ? edgeCur->right_next->end : edgeCur->left_next->end;
            auto vertexTopLeft = j == 0 ? edgeCur->right_next->left_next->end : edgeCur->left_next->left_next->end;
            auto vertexTopRight = j == 0 ? edgeCur->right_prev->left_next->end : edgeCur->left_prev->left_next->end;
            auto vertexBotMid = j == 0 ? edgeCur->left_next->end : edgeCur->right_next->end;
            auto vertexBotLeft = j == 0 ? edgeCur->left_prev->left_next->end : edgeCur->right_prev->left_next->end;
            auto vertexBotRight = j == 0 ? edgeCur->left_next->left_next->end : edgeCur->right_next->left_next->end;
            int vertexMidLeftIndex = vertexMidLeft->index;
            int vertexMidRightIndex = vertexMidRight->index;
            float newX = 0, newY = 0, newZ = 0;
            auto midLeftNeighbours = getNeighbourVertices(vertexMidLeft);
            int vertexMidLeftValence = midLeftNeighbours.size();
            auto midRightNeighbours = getNeighbourVertices(vertexMidRight);
            int vertexMidRightValence = midRightNeighbours.size();

            // cout << "left K: " << vertexMidLeftValence << endl;
            // cout << "right K: " << vertexMidRightValence << endl;
            
            if(vertexMidLeftValence == REGLUAR_VALENCE && vertexMidRightValence == REGLUAR_VALENCE)
            {
                newX = 0.5 * vertexMidLeft->x + 0.5 * vertexMidRight->x - 1.0 / 16 * vertexTopLeft->x + 1.0 / 8 * vertexTopMid->x - 1.0 / 16 * vertexTopRight->x 
                        - 1.0 / 16 * vertexBotLeft->x + 1.0 / 8 * vertexBotMid->x - 1.0 / 16 * vertexBotRight->x;
                newY = 0.5 * vertexMidLeft->y + 0.5 * vertexMidRight->y - 1.0 / 16 * vertexTopLeft->y + 1.0 / 8 * vertexTopMid->y - 1.0 / 16 * vertexTopRight->y
                        - 1.0 / 16 * vertexBotLeft->y + 1.0 / 8 * vertexBotMid->y - 1.0 / 16 * vertexBotRight->y;
                newZ = 0.5 * vertexMidLeft->z + 0.5 * vertexMidRight->z - 1.0 / 16 * vertexTopLeft->z + 1.0 / 8 * vertexTopMid->z - 1.0 / 16 * vertexTopRight->z
                        - 1.0 / 16 * vertexBotLeft->z + 1.0 / 8 * vertexBotMid->z - 1.0 / 16 * vertexBotRight->z;
            }
            else
            {
                float newX_left = 0, newY_left = 0, newZ_left = 0;
                float newX_right = 0, newY_right = 0, newZ_right = 0;

                // calculate left vertex
                if(vertexMidLeftValence == 3)
                {
                    auto neighbour0 = vertexMidRight;
                    auto neighbour1 = j == 0 ? edgeCur->right_next->end : edgeCur->left_next->end;
                    auto neighbour2 = j == 0 ? edgeCur->left_next->end : edgeCur->right_next->end;
                    newX_left = 0.75 * vertexMidLeft->x + 5.0 / 12 * neighbour0->x - 1.0 / 12 * neighbour1->x - 1.0 / 12 * neighbour2->x;
                    newY_left = 0.75 * vertexMidLeft->y + 5.0 / 12 * neighbour0->y - 1.0 / 12 * neighbour1->y - 1.0 / 12 * neighbour2->y;
                    newZ_left = 0.75 * vertexMidLeft->z + 5.0 / 12 * neighbour0->z - 1.0 / 12 * neighbour1->z - 1.0 / 12 * neighbour2->z;
                }
                else if(vertexMidLeftValence == 4)
                {
                    auto neighbour0 = vertexMidRight;
                    auto neighbour2 = j == 0 ? edgeCur->right_next->left_next->end : edgeCur->left_next->left_next->end;
                    newX_left = 0.75 * vertexMidLeft->x + 3.0 / 8 * neighbour0->x - 1.0 / 8 * neighbour2->x;
                    newY_left = 0.75 * vertexMidLeft->y + 3.0 / 8 * neighbour0->y - 1.0 / 8 * neighbour2->y;
                    newZ_left = 0.75 * vertexMidLeft->z + 3.0 / 8 * neighbour0->z - 1.0 / 8 * neighbour2->z;
                }
                else if(vertexMidLeftValence > 4)
                {
                    vector<float> weights;
                    float selfWeight = 1.0;
                    for(int n = 0; n < vertexMidLeftValence; n++)
                    {
                        float curWeight = (1.0 / vertexMidLeftValence) * (0.25 + cos(2 * n * M_PI / vertexMidLeftValence) + 0.5 * cos(4 * n * M_PI / vertexMidLeftValence));
                        weights.push_back(curWeight);
                        selfWeight -= curWeight;
                    }

                    int startPos = 0;
                    for(int n = 0; n < midLeftNeighbours.size(); n++)
                    {                    
                        if(midLeftNeighbours.at(n) == vertexMidRight)
                        {
                            startPos = n;
                            break;
                        }
                    }                

                    for(int n = 0; n < vertexMidLeftValence; n++)
                    {
                        newX_left += weights.at(n) * midLeftNeighbours.at((startPos + n) % weights.size())->x;
                        newY_left += weights.at(n) * midLeftNeighbours.at((startPos + n) % weights.size())->y;
                        newZ_left += weights.at(n) * midLeftNeighbours.at((startPos + n) % weights.size())->z;
                    }      
                    newX_left += selfWeight * vertexMidLeft->x;
                    newY_left += selfWeight * vertexMidLeft->y;
                    newZ_left += selfWeight * vertexMidLeft->z;
                }

                // calculate right vertex
                if(vertexMidRightValence == 3)
                {
                    auto neighbour0 = vertexMidLeft;
                    auto neighbour1 = j == 0 ? edgeCur->left_next->end : edgeCur->right_next->end;
                    auto neighbour2 = j == 0 ? edgeCur->right_next->end : edgeCur->left_next->end;
                    newX_right = 0.75 * vertexMidRight->x + 5.0 / 12 * neighbour0->x - 1.0 / 12 * neighbour1->x - 1.0 / 12 * neighbour2->x;
                    newY_right = 0.75 * vertexMidRight->y + 5.0 / 12 * neighbour0->y - 1.0 / 12 * neighbour1->y - 1.0 / 12 * neighbour2->y;
                    newZ_right = 0.75 * vertexMidRight->z + 5.0 / 12 * neighbour0->z - 1.0 / 12 * neighbour1->z - 1.0 / 12 * neighbour2->z;
                }
                else if(vertexMidRightValence == 4)
                {
                    auto neighbour0 = vertexMidLeft;
                    auto neighbour2 = j == 0 ? edgeCur->left_next->left_next->end : edgeCur->right_next->left_next->end;
                    newX_right = 0.75 * vertexMidRight->x + 3.0 / 8 * neighbour0->x - 1.0 / 8 * neighbour2->x;   
                    newY_right = 0.75 * vertexMidRight->y + 3.0 / 8 * neighbour0->y - 1.0 / 8 * neighbour2->y;
                    newZ_right = 0.75 * vertexMidRight->z + 3.0 / 8 * neighbour0->z - 1.0 / 8 * neighbour2->z;
                }
                else if(vertexMidRightValence > 4)
                {
                    vector<float> weights;
                    float selfWeight = 1.0;
                    for(int n = 0; n < vertexMidRightValence; n++)
                    {
                        float curWeight = (1.0 / vertexMidRightValence) * (0.25 + cos(2 * n * M_PI / vertexMidRightValence) + 0.5 * cos(4 * n * M_PI / vertexMidRightValence));
                        weights.push_back(curWeight);
                        selfWeight -= curWeight;
                    }

                    int startPos = 0;
                    for(int n = 0; n < midRightNeighbours.size(); n++)
                    {                    
                        if(midRightNeighbours.at(n) == vertexMidLeft)
                        {
                            startPos = n;
                            break;
                        }
                    }                

                    for(int n = 0; n < vertexMidRightValence; n++)
                    {
                        newX_right += weights.at(n) * midRightNeighbours.at((startPos + n) % weights.size())->x;
                        newY_right += weights.at(n) * midRightNeighbours.at((startPos + n) % weights.size())->y;
                        newZ_right += weights.at(n) * midRightNeighbours.at((startPos + n) % weights.size())->z;
                    }      
                    newX_right += selfWeight * vertexMidRight->x;
                    newY_right += selfWeight * vertexMidRight->y;
                    newZ_right += selfWeight * vertexMidRight->z;
                }

                if(vertexMidRightValence == REGLUAR_VALENCE)
                {
                    newX = newX_left;
                    newY = newY_left;
                    newZ = newZ_left;
                }
                else if(vertexMidLeftValence == REGLUAR_VALENCE)
                {
                    newX = newX_right;
                    newY = newY_right;
                    newZ = newZ_right;
                }
                else if(vertexMidLeftValence != REGLUAR_VALENCE && vertexMidRightValence != REGLUAR_VALENCE)
                {
                    newX = 0.5 * (newX_left + newX_right);
                    newY = 0.5 * (newY_left + newY_right);
                    newZ = 0.5 * (newZ_left + newZ_right);
                }
            }

            EdgeVertexIndices indices = make_pair(vertexMidLeftIndex, vertexMidRightIndex);
            EdgeIndicesToIndexMap::const_iterator itr = edgeToNewVertexIndex.find(indices);
            if (itr == edgeToNewVertexIndex.end())
            {
                newVerticesIndices.push_back(newVertexIndex);
                EdgeVertexIndices reverseIndices = make_pair(vertexMidRightIndex, vertexMidLeftIndex);
                edgeToNewVertexIndex[indices] = newVertexIndex;
                edgeToNewVertexIndex[reverseIndices] = newVertexIndex;            
                newVertices.col(newVertexIndex) << newX, newY, newZ;
                newVertexIndex++;
            }
            else
            {
                newVerticesIndices.push_back(edgeToNewVertexIndex[indices]);
            }
        }        

        int newVertex1Index = newVerticesIndices.at(0);
        int newVertex2Index = newVerticesIndices.at(1);
        int newVertex3Index = newVerticesIndices.at(2);

        newFaces.col(4 * i) << v2Index, newVertex3Index, newVertex1Index;
        newFaces.col(4 * i + 1) << newVertex1Index, newVertex3Index, newVertex2Index;
        newFaces.col(4 * i + 2) << newVertex1Index, newVertex2Index, v1Index;
        newFaces.col(4 * i + 3) << newVertex3Index, v3Index, newVertex2Index;
    }

    // ofstream newFile("test.obj");
    // cout << "saving" << endl;
    // if(newFile.is_open())   
    // {
    //     int numFaces = newFaces.cols();
    //     int numVertices = newVertices.cols();

    //     newFile << "# " << numVertices << " " << numFaces << endl;

    //     for(int i = 0; i < newVertices.cols(); i++)
    //     {
    //         newFile << "v " << newVertices.col(i).x() << " " << newVertices.col(i).y() << " " << newVertices.col(i).z() << endl;
    //     }

    //     for(int i = 0; i < newFaces.cols(); i++)
    //     {
    //         newFile << "f " << newFaces.col(i).x() + 1 << " " << newFaces.col(i).y() + 1 << " " << newFaces.col(i).z() + 1 << endl;
    //     }
    // }
    // else 
    // {
    //     cout << "Failed to open file for saving: " << "test.obj" << endl;
    // }

    // newFile.close();
    // cout << "saved" << endl;

    // update
    updateFromMats(newVertices, newFaces);
}

void WingedEdgeMesh::decimate(const int &k, const int &numEdges)
{
    updateFaceKp();
    updateVertexQ();
    
    int decimatedCount = 0;
    const int maxConsecutiveFails = 50;
    int consecutiveFails = 0;
    while(decimatedCount < numEdges)
    {
        if(consecutiveFails >= maxConsecutiveFails)
        {
            cout << "Error: Cannot find valid edges to continue decimating." << endl;
            return;
        }
        
        // simpliest mesh tetrahedron requires 6 edges, and winged edge stores each edge in 2 directions
        if(mEdges.size() <= 6 * 2)
        {
            cout << "Error: Not enough edges to continue decimating." << endl;
            return;
        }

        // cout << "initial vertices: " << mVertices.size() << endl;
        // cout << "initial faces: " << mFaces.size() << endl;
        // cout << "initial edges: " << mEdges.size() << endl;
        Eigen::MatrixXf newVertices(3, mVertices.size() - 1);
        Eigen::MatrixXi newFaces(3, 0);
        
        shared_ptr<W_edge> edgeToDecimate;
        Vector4f newV;
        getMultipleChoiceBestEdge(k, edgeToDecimate, newV);
        auto v1 = edgeToDecimate->start;
        auto v2 = edgeToDecimate->end;

        // if the two vertices on the edge share more than 2 neighbour vertices, then do not decimate
        // otherwise would create non-manifold surface
        auto v1Neighbours = getNeighbourVertices(v1);
        auto v2Neighbours = getNeighbourVertices(v2);
        vector<shared_ptr<Vertex>> sharedNeighbours;
        sort(v1Neighbours.begin(), v1Neighbours.end());
        sort(v2Neighbours.begin(), v2Neighbours.end());
        set_intersection(v1Neighbours.begin(), v1Neighbours.end(), v2Neighbours.begin(), v2Neighbours.end(), back_inserter(sharedNeighbours));
        // cout << "sharedNeighbours: " << sharedNeighbours.size() << endl;
        if(sharedNeighbours.size() > 2)
        {
            consecutiveFails++;
            continue;
        }
        
        v1->x = newV(0);
        v1->y = newV(1);
        v1->z = newV(2);
        v1->Q = v1->Q + v2->Q;

        vector<Eigen::Matrix4f> Qs;
        int newVertexIndex = 0;
        for (int i = 0; i < mVertices.size(); i++)
        {
            auto vertex = mVertices.at(i);
            if(vertex->index != v2->index)        
            {
                newVertices.col(newVertexIndex) << vertex->x, vertex->y, vertex->z;
                Qs.push_back(vertex->Q);
                newVertexIndex++;
            }
        }
        
        bool skipToNextInteration = false;
        for (int i = 0; i < mFaces.size(); i++)
        {
            auto faceVertex1 = mFaces.at(i)->edge->start;
            auto faceVertex2 = mFaces.at(i)->edge->end;
            auto faceVertex3 = mFaces.at(i)->edge->left_next->end;

            // calculate old face normal
            Vector3f vec1 = {faceVertex2->x - faceVertex1->x,
                        faceVertex2->y - faceVertex1->y, 
                        faceVertex2->z - faceVertex1->z};
            Vector3f vec2 = {faceVertex3->x - faceVertex1->x, 
                        faceVertex3->y - faceVertex1->y, 
                        faceVertex3->z - faceVertex1->z};
            Vector3f faceNormal = vec1.cross(vec2).normalized();

            vector<int> faceVertexIndices;
            faceVertexIndices.push_back(faceVertex1->index);
            faceVertexIndices.push_back(faceVertex2->index);
            faceVertexIndices.push_back(faceVertex3->index);
            
            // remove faces that contain both vertices of the edge to be decimated
            if(vectorContains(faceVertexIndices, v1->index) && vectorContains(faceVertexIndices, v2->index))
            {
                continue;
            }

            // transfer v2's faces to v1            
            int newV1Index = v1->index > v2->index ? v1->index - 1 : v1->index;
            int newFV1Index = faceVertex1->index > v2->index ? faceVertex1->index - 1 : faceVertex1->index;
            int newFV2Index = faceVertex2->index > v2->index ? faceVertex2->index - 1 : faceVertex2->index;
            int newFV3Index = faceVertex3->index > v2->index ? faceVertex3->index - 1 : faceVertex3->index;
            int outputIndex1 = 0, outputIndex2 = 0, outputIndex3 = 0;
            if(vectorContains(faceVertexIndices, v2->index))
            {
                if(faceVertex1 == v2)
                {
                    outputIndex1 = newV1Index;
                    outputIndex2 = newFV2Index;
                    outputIndex3 = newFV3Index;                    
                }
                else if(faceVertex2 == v2)
                {
                    outputIndex1 = newFV1Index;
                    outputIndex2 = newV1Index;
                    outputIndex3 = newFV3Index;   
                }
                else if(faceVertex3 == v2)
                {
                    outputIndex1 = newFV1Index;
                    outputIndex2 = newFV2Index;
                    outputIndex3 = newV1Index;  
                }
            }
            else
            {
                outputIndex1 = newFV1Index;
                outputIndex2 = newFV2Index;
                outputIndex3 = newFV3Index;
            }

            // calculate new face normal
            Vector3f newFaceVertex1(newVertices.col(outputIndex1));
            Vector3f newFaceVertex2(newVertices.col(outputIndex2));
            Vector3f newFaceVertex3(newVertices.col(outputIndex3));
            Vector3f newVec1 = {newFaceVertex2.x() - newFaceVertex1.x(),
                        newFaceVertex2.y() - newFaceVertex1.y(), 
                        newFaceVertex2.z() - newFaceVertex1.z()};
            Vector3f newVec2 = {newFaceVertex3.x() - newFaceVertex1.x(), 
                        newFaceVertex3.y() - newFaceVertex1.y(), 
                        newFaceVertex3.z() - newFaceVertex1.z()};
            Vector3f newFaceNormal = newVec1.cross(newVec2).normalized();

            // fix float precision errors
            float normalsDotProduct = faceNormal.dot(newFaceNormal);
            if (normalsDotProduct > 1)
            {
                normalsDotProduct = 1;
            }
            else if (normalsDotProduct < -1)
            {
                normalsDotProduct = -1;
            }

            float normalAngleChange = std::acos(normalsDotProduct) * 180. / M_PI;
            
            // if change in face normal is too big then do not decimate the edge
            if(normalAngleChange > 90)
            {
                skipToNextInteration = true;
                consecutiveFails++;
                break;
            }

            newFaces.conservativeResize(newFaces.rows(), newFaces.cols() + 1);
            newFaces.col(newFaces.cols() - 1) << outputIndex1, outputIndex2, outputIndex3;
        }

        if(skipToNextInteration)
        {
            continue;
        }

        // cout << "new vertices: " << newVertices.cols() << endl;
        // cout << "new faces: " << newFaces.cols() << endl;
        updateFromMats(newVertices, newFaces);
        for(int i = 0; i < mVertices.size(); i++)
        {
            mVertices.at(i)->Q = Qs.at(i);
        }
        decimatedCount++;
        consecutiveFails = 0;
    }
}

const vector<shared_ptr<Vertex>>& WingedEdgeMesh::getVertices()
{
    return mVertices;
}

const vector<shared_ptr<Face>>& WingedEdgeMesh::getFaces()
{
    return mFaces;
}

const EdgesHashMap& WingedEdgeMesh::getEdges()
{
    return mEdges;
}

vector<shared_ptr<Vertex>> WingedEdgeMesh::getNeighbourVertices(shared_ptr<Vertex> vertex)
{
    auto e0 = vertex->edge;
    auto edge = e0;
    vector<shared_ptr<Vertex>> neighbourVertices;
    int maxLoopCount = 100;
    int curLoopCount = 0;
    do
    {            
        if(curLoopCount >= maxLoopCount)
        {
            cout << "WingedEdgeMesh::getNeighbourVertices maxLoopCount exceeded." << endl;
            break;
        }
        neighbourVertices.push_back(edge->end);
        edge = edge->left_next;
        curLoopCount++;
    }while(edge != e0);

    return neighbourVertices;
}

vector<shared_ptr<Face>> WingedEdgeMesh::getNeighbourFaces(shared_ptr<Vertex> vertex)
{
    auto e0 = vertex->edge;
    auto edge = e0;
    vector<shared_ptr<Face>> neighbourFaces;
    int maxLoopCount = 100;
    int curLoopCount = 0;
    do
    {            
        if(curLoopCount >= maxLoopCount)
        {
            cout << "WingedEdgeMesh::getNeighbourVertices maxLoopCount exceeded." << endl;
            break;
        }
        neighbourFaces.push_back(edge->left);
        edge = edge->left_next;
        curLoopCount++;
    }while(edge != e0);

    return neighbourFaces;
}

void WingedEdgeMesh::updateFaceKp()
{
    for (int i = 0; i < mFaces.size(); i++)
    {
        auto face = mFaces.at(i);
        auto v1 = face->edge->start;
        auto v2 = face->edge->end;
        auto v3 = face->edge->left_next->end;

        Vector3f vec1 = {v2->x - v1->x,
                        v2->y - v1->y, 
                        v2->z - v1->z};
        Vector3f vec2 = {v3->x - v1->x, 
                        v3->y - v1->y, 
                        v3->z - v1->z};
        Vector3f faceNormal = vec1.cross(vec2).normalized();

        float a = faceNormal[0];
        float b = faceNormal[1];
        float c = faceNormal[2];
        float d = -1 * a * v1->x - b * v1->y - c * v1->z;
        face->Kp.col(0) << a * a, a * b, a * c, a * d;
        face->Kp.col(1) << a * b, b * b, b * c, b * d;
        face->Kp.col(2) << a * c, b * c, c * c, c * d;
        face->Kp.col(3) << a * d, b * d, c * d, d * d;
    }
}

void WingedEdgeMesh::updateVertexQ()
{
    for (int i = 0; i < mVertices.size(); i++)
    {
        auto vertex = mVertices.at(i);
        auto neighbourFaces = getNeighbourFaces(vertex);
        Eigen::Matrix4f Kp_sum;
        vertex->Q.setZero();
        for (int j = 0; j < neighbourFaces.size(); j++)
        {
            auto face = neighbourFaces.at(j);
            vertex->Q += face->Kp;
        }
    }
}

void WingedEdgeMesh::getMultipleChoiceBestEdge(const int &k, shared_ptr<W_edge> &bestEdge, Vector4f &newV)
{
    int numEdges = mEdges.size();
    int kTemp = k;
    if(k > numEdges)
    {
        kTemp = numEdges;
    }
    random_device rd;  // obtain a random number from hardware
    mt19937 eng(rd()); // seed the generator
    uniform_int_distribution<> distr(0, numEdges - 1); // define the range
    int count = 0;
    vector<int> selected;
    while(count < kTemp)
    {
        int randomNumber = distr(eng);
        bool exists = vectorContains(selected, randomNumber);
        if(!exists)
        {
            selected.push_back(randomNumber);
            count++;
        }
    }

    int curIndex = 0;
    float minError = 99999;
    for (auto i : mEdges)
    {
        if(vectorContains(selected, curIndex))
        {
            shared_ptr<W_edge> edge = i.second;
            auto v1 = edge->start;
            auto v2 = edge->end;
            auto Q_sum = v1->Q + v2->Q;

            Eigen::Matrix4f A;
            A(0, 0) = Q_sum(0, 0);
            A(1, 0) = Q_sum(0, 1);
            A(2, 0) = Q_sum(0, 2);
            A(3, 0) = 0;
            A(0, 1) = Q_sum(0, 1);
            A(1, 1) = Q_sum(1, 1);
            A(2, 1) = Q_sum(1, 2);
            A(3, 1) = 0;
            A(0, 2) = Q_sum(0, 2);
            A(1, 2) = Q_sum(1, 2);
            A(2, 2) = Q_sum(2, 2);
            A(3, 2) = 0;
            A(0, 3) = Q_sum(0, 3);
            A(1, 3) = Q_sum(1, 3);
            A(2, 3) = Q_sum(2, 3);
            A(3, 3) = 1;

            Vector4f b(0, 0, 0, 1);
            Vector4f optimalV = A.inverse() * b;
            float error = optimalV.transpose() * Q_sum * optimalV;
            if(error < minError)
            {
                minError = error;
                bestEdge = edge;
                newV = optimalV;
            }
        }
        curIndex++;
    }
}

bool WingedEdgeMesh::vectorContains(const vector<int> vec, int num)
{
    for(int i = 0; i < vec.size(); i++)
    {
        if(vec.at(i) == num)
        {
            return true;
        }
    }
    return false;
}

void WingedEdgeMesh::clear()
{
    for(auto edge : mEdges)
    {
        edge.second->start.reset();
        edge.second->end.reset();
        edge.second->left.reset();
        edge.second->right.reset();
        edge.second->left_prev.reset();
        edge.second->left_next.reset();
        edge.second->right_prev.reset();
        edge.second->right_next.reset();
        edge.second.reset();
    }

    for(auto vertex : mVertices)
    {
        vertex->edge.reset();
        vertex.reset();
    }

    for(auto face : mFaces)
    {
        face->edge.reset();
        face.reset();
    }

    mEdges.clear();
    mVertices.clear();
    mFaces.clear();
}

vector<string> WingedEdgeMesh::split(const string &str, const string &delim)
{
    char* cstr = const_cast<char*>(str.c_str());
    char* current;
    vector<string> arr;
    current = strtok(cstr, delim.c_str());
    while(current != NULL){
        arr.push_back(current);
        current = strtok(NULL, delim.c_str());
    }
    return arr;
}