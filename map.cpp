#include "map.h"
using namespace tinyxml2;

Map::Map()
{
    height = 0;
    width = 0;
    length = 0;
}
Map::~Map()
{	
    Grid.clear();
}

bool Map::getMap(const char* FileName)
{
    XMLDocument doc;
    if(doc.LoadFile(FileName) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }

    XMLElement *root = nullptr;
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML map-file."<<std::endl;
        return false;
    }

    XMLElement *map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "No '"<<CNS_TAG_MAP<<"' element found in XML file."<<std::endl;
        return false;
    }

    XMLElement *grid = map->FirstChildElement(CNS_TAG_GRID);
    if (!grid)
    {
        std::cout << "No '"<<CNS_TAG_GRID<<"' element found in XML file."<<std::endl;
        return false;
    }
    height = grid->IntAttribute(CNS_TAG_ATTR_HEIGHT);
    if(height <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_HEIGHT<<" attribute. It should be >0.\n";
        return false;
    }
    width = grid->IntAttribute(CNS_TAG_ATTR_WIDTH);
    if(width <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_WIDTH<<" attribute. It should be >0.\n";
        return false;
    }
    length = grid->IntAttribute(CNS_TAG_ATTR_LENGTH);
    if (length <= 0) {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_LENGTH<<" attribute. It should be >0.\n";
        return false;
    }
    XMLElement *plane = grid->FirstChildElement(CNS_TAG_PLANE);
    XMLElement *row = nullptr;
    Grid.resize(length);
    for(int i = 0; i < length; i++) {
        Grid[i].resize(height);
        for (int j = 0; j < height; j++) {
            Grid[i][j].resize(width, 0);
        }
    }

    std::string value;
    const char* rowtext;
    std::stringstream stream;
    for(int p = 0; p < length; p++)
    {
        if (!plane)
        {
            std::cout << "Not enough '" << CNS_TAG_PLANE << "' in '" << CNS_TAG_GRID << "' given." << std::endl;
            return false;
        }
        row = plane->FirstChildElement(CNS_TAG_ROW);
        for (int r = 0; r < height; r++) {
            if (!row)
            {
                std::cout << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_PLANE << "' " << p << " given." << std::endl;
                return false;
            }
            rowtext = row->GetText();
            unsigned int k = 0;
            value = "";
            int c = 0;

            for (k = 0; k < strlen(rowtext); k++) {
                if (rowtext[k] == ' ') {
                    stream << value;
                    stream >> Grid[p][r][c];
                    stream.clear();
                    stream.str("");
                    value = "";
                    c++;
                } else {
                    value += rowtext[k];
                }
            }
            stream << value;
            stream >> Grid[p][r][c];
            stream.clear();
            stream.str("");

            if (c < width - 1) {
                std::cout << "Not enough cells in '" << CNS_TAG_ROW << "' " << r << " given." << std::endl;
                return false;
            }
            row = row->NextSiblingElement(CNS_TAG_ROW);
        }
        plane = plane->NextSiblingElement(CNS_TAG_PLANE);
    }
    return true;
}


bool Map::CellIsTraversable(int i, int j, int k) const
{
    return (Grid[i][j][k] == 0);
}

bool Map::CellIsObstacle(int i, int j, int k) const
{
    return (Grid[i][j][k] != 0);
}

bool Map::CellOnGrid(int i, int j, int k) const
{
    return (i < length && i >= 0 && j < height && j >= 0 && k < width && k >= 0);
}

int Map::getValue(int i, int j, int k) const
{
    if(i < 0 || i >= length)
        return -1;
    if(j < 0 || j >= height)
        return -1;
    if(k < 0 || k >= width)
        return -1;

    return Grid[i][j][k];
}

std::vector<Node> Map::getValidMoves(int i, int j, int k, int neig_num, double size) const
{
   LineOfSight los;
   los.setSize(size);
   std::vector<Node> moves;
   if(neig_num == 2)
       moves = {Node(0,0,1,1.0), Node(0,1,0,1.0), Node(0,-1,0,1.0), Node(0,0,-1,1.0), Node(1,0,0,1.0), Node(-1,0,0,1.0)};
   else if(neig_num == 3)
       moves = {Node(0,0,1,1.0),          Node(0,1,1,sqrt(2.0)),    Node(0,1,0,1.0),         Node(0,1,-1,sqrt(2.0)),
                Node(0,0,-1,1.0),         Node(0,-1,-1,sqrt(2.0)),  Node(0,-1,0,1.0),        Node(0,-1,1,sqrt(2.0)),
                Node(1,0,0,1.0),
                Node(1,0,1,sqrt(2.0)),    Node(1,1,1,sqrt(3.0)),    Node(1,1,0,sqrt(2.0)),   Node(1,1,-1,sqrt(3.0)),
                Node(1,0,-1,sqrt(2.0)),   Node(1,-1,-1,sqrt(3.0)),  Node(1,-1,0,sqrt(2.0)),  Node(1,-1,1,sqrt(3.0)),
                Node(-1,0,0,1.0),
                Node(-1,0,1,sqrt(2.0)),   Node(-1,1,1,sqrt(3.0)),   Node(-1,1,0,sqrt(2.0)),  Node(-1,1,-1,sqrt(3.0)),
                Node(-1,0,-1,sqrt(2.0)),  Node(-1,-1,-1,sqrt(3.0)), Node(-1,-1,0,sqrt(2.0)), Node(-1,-1,1,sqrt(3.0))};
   // else if(neig_num == 4)
   //     moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
   //              Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
   //              Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
   //              Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0))};
   // else
   //     moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
   //              Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
   //              Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
   //              Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0)),
   //              Node(1,3,sqrt(10.0)),   Node(2,3,sqrt(13.0)),   Node(3,2,sqrt(13.0)),   Node(3,1,sqrt(10.0)),
   //              Node(3,-1,sqrt(10.0)),  Node(3,-2,sqrt(13.0)),  Node(2,-3,sqrt(13.0)),  Node(1,-3,sqrt(10.0)),
   //              Node(-1,-3,sqrt(10.0)), Node(-2,-3,sqrt(13.0)), Node(-3,-2,sqrt(13.0)), Node(-3,-1,sqrt(10.0)),
   //              Node(-3,1,sqrt(10.0)),  Node(-3,2,sqrt(13.0)),  Node(-2,3,sqrt(13.0)),  Node(-1,3,sqrt(10.0))};
   std::vector<bool> valid(moves.size(), true);
   for(int num = 0; num < moves.size(); num++)
       if(!CellOnGrid(i + moves[num].i, j + moves[num].j, k + moves[num].k)
       || CellIsObstacle(i + moves[num].i, j + moves[num].j, k + moves[num].k)
       || !los.checkLine(i, j, k, i + moves[num].i, j + moves[num].j, k + moves[num].k, *this))
           valid[num] = false;
   std::vector<Node> v_moves = {};
   for(int num = 0; num < valid.size(); num++)
       if(valid[num])
           v_moves.push_back(moves[num]);
   return v_moves;
}
