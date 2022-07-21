#include "xmlser.h"

CXmlSer::CXmlSer(const char* filename)
{
    currentNode=nullptr;
    _filename=filename;
    xmlNode* mainNode=_createNode("mujoco");
    _pushNode(mainNode);
    setAttr("model","coppeliaSim");
}


CXmlSer::~CXmlSer()
{
    _document.SaveFile(_filename.c_str(),false);
}

xmlNode* CXmlSer::_createNode(const char* name)
{
    xmlNode* node=_document.NewElement(name);
    return(node);
}

void CXmlSer::pushNewNode(const char* name)
{
    xmlNode* node=_createNode(name);
    _pushNode(node);
}

void CXmlSer::_pushNode(xmlNode* node)
{
    if (currentNode==nullptr)
        _document.InsertFirstChild(node);
    else
        currentNode->InsertEndChild(node);
    currentNode=node;
    _nodes.push_back(node);
}

void CXmlSer::popNode()
{
    if (_nodes.size()>0)
    {
        _nodes.pop_back();
        if (_nodes.size()>0)
            currentNode=_nodes[_nodes.size()-1];
        else
            currentNode=nullptr;
    }
}

void CXmlSer::setAttr(const char* name,const char* value)
{
    currentNode->SetAttribute(name,value);
}

void CXmlSer::setAttr(const char* name,bool value)
{
    if (value)
        setAttr(name,"true");
    else
        setAttr(name,"false");
}

void CXmlSer::setAttr(const char* name,int value)
{
    currentNode->SetAttribute(name,value);
}

void CXmlSer::setAttr(const char* name,int value1,int value2)
{
    std::string tmp(std::to_string(value1)+" "+std::to_string(value2));
    setAttr(name,tmp.c_str());
}

void CXmlSer::setAttr(const char* name,int value1,int value2,int value3)
{
    std::string tmp(std::to_string(value1)+" "+std::to_string(value2)+" "+std::to_string(value3));
    setAttr(name,tmp.c_str());
}

void CXmlSer::setAttr(const char* name,const int* value,size_t size)
{
    std::string tmp(std::to_string(value[0]));
    for (size_t i=1;i<size;i++)
        tmp+=" "+std::to_string(value[i]);
    setAttr(name,tmp.c_str());
}

void CXmlSer::setAttr(const char* name,double value)
{
    currentNode->SetAttribute(name,value);
}

void CXmlSer::setAttr(const char* name,double value1,double value2)
{
    std::string tmp(std::to_string(value1)+" "+std::to_string(value2));
    setAttr(name,tmp.c_str());
}

void CXmlSer::setAttr(const char* name,double value1,double value2,double value3)
{
    std::string tmp(std::to_string(value1)+" "+std::to_string(value2)+" "+std::to_string(value3));
    setAttr(name,tmp.c_str());
}

void CXmlSer::setAttr(const char* name,double value1,double value2,double value3,double value4)
{
    std::string tmp(std::to_string(value1)+" "+std::to_string(value2)+" "+std::to_string(value3)+" "+std::to_string(value4));
    setAttr(name,tmp.c_str());
}

void CXmlSer::setAttr(const char* name,double value1,double value2,double value3,double value4,double value5)
{
    std::string tmp(std::to_string(value1)+" "+std::to_string(value2)+" "+std::to_string(value3)+" "+std::to_string(value4)+" "+std::to_string(value5));
    setAttr(name,tmp.c_str());
}

void CXmlSer::setAttr(const char* name,const double* value,size_t size)
{
    std::string tmp(std::to_string(value[0]));
    for (size_t i=1;i<size;i++)
        tmp+=" "+std::to_string(value[i]);
    setAttr(name,tmp.c_str());
}

void CXmlSer::setPosAttr(const char* name,const float value[3])
{
    std::string tmp(std::to_string(value[0])+" "+std::to_string(value[1])+" "+std::to_string(value[2]));
    setAttr(name,tmp.c_str());
}

void CXmlSer::setQuatAttr(const char* name,const float value[4])
{
    std::string tmp(std::to_string(value[0])+" "+std::to_string(value[1])+" "+std::to_string(value[2])+" "+std::to_string(value[3]));
    setAttr(name,tmp.c_str());
}

