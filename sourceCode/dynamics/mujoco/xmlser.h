#pragma once

#include "tinyxml2.h"
#include <string>
#include <vector>

typedef sim::tinyxml2::XMLElement xmlNode;

class CXmlSer
{
public:
    CXmlSer(const char* filename);

    virtual ~CXmlSer();

    void pushNewNode(const char* name);
    void popNode();
    
    void setAttr(const char* name,const char* value);

    void setAttr(const char* name,bool value);

    void setAttr(const char* name,int value);
    void setAttr(const char* name,int value1,int value2);
    void setAttr(const char* name,int value1,int value2,int value3);
    void setAttr(const char* name,const int* value,size_t size);

    void setAttr(const char* name,double value);
    void setAttr(const char* name,double value1,double value2);
    void setAttr(const char* name,double value1,double value2,double value3);
    void setAttr(const char* name,double value1,double value2,double value3,double value4);
    void setAttr(const char* name,double value1,double value2,double value3,double value4,double value5);
    void setAttr(const char* name,const double* value,size_t size);
    
    void setPosAttr(const char* name,const float value[3]);
    void setQuatAttr(const char* name,const float value[4]);

    xmlNode* currentNode;
    
private:
    xmlNode* _createNode(const char* name);
    void _pushNode(xmlNode* node);
    
    sim::tinyxml2::XMLDocument _document;
    std::vector<xmlNode*> _nodes;
    std::string _filename;
};
