#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <vector>
#include "tinyxml.h"
#include "tinystr.h"

using namespace std;

class _Point
{
public:
    int x, y;
    _Point(int x=0, int y=0) {
        this->x = x;
        this->y = y;
    }
};

class Aera
{
private:
    char T_string[100];
public:
    int id;
    double T[6];
    vector<_Point> points;
    Aera(int id) {
        this->id = id;
    }
    char* getT() {
        sprintf(T_string, "%f %f %f %f %f %f", T[0], T[1], T[2], T[3], T[4], T[5]);
        return T_string;
    }
    int setT(double T00, double T01, double T10, double T11, double T02, double T12) {
        T[0] = T00;
        T[1] = T01;
        T[2] = T10;
        T[3] = T11;
        T[4] = T02;
        T[5] = T12;
        return 0;
    }
    int setT(double T[]) {
        for (int i=0; i<6; i++) {
            this->T[i] = T[i];
        }
        return 0;
    }
    bool inAera(_Point p) {
        int crossings = 0;
        for (int i = 0, j = points.size()-1; i < points.size(); j = i++) {
            int xi = points[i].x;
            int xj = points[j].x;
            int yi = points[i].y;
            int yj = points[j].y;
            if ((((xi < p.x) && (xj >= p.x)) || ((xi >= p.x) && (xj < p.x)))
                && (p.x > (xj - xi) * (p.y - yi) / (yj - yi) + xi))
            crossings++;
        }
        return (crossings % 2 != 0);
    }
    _Point transform(_Point p) {
        int x = T[0] * p.x + T[1] * p.y + T[4];
        int y = T[2] * p.x + T[3] * p.y + T[5];
        return _Point(x, y);
    }
};

class Map
{
public:
    vector<Aera> areas;
};

bool CreateXmlFile(string& szFileName, Map map)
{
    try
    {
        TiXmlDocument *myDocument = new TiXmlDocument();
        TiXmlElement *RootElement = new TiXmlElement("Map");
        myDocument->LinkEndChild(RootElement);
        for (auto area : map.areas) {
            TiXmlElement *aera_element = new TiXmlElement("Aera");
            RootElement->LinkEndChild(aera_element);
            aera_element->SetAttribute("id", area.id);
            aera_element->SetAttribute("T", area.getT());
            for (auto point : area.points) {
                TiXmlElement *point_element = new TiXmlElement("_Point");
                aera_element->LinkEndChild(point_element);
                point_element->SetAttribute("x", point.x);
                point_element->SetAttribute("y", point.y);
            }
        }
        myDocument->SaveFile(szFileName.c_str());
        cout << szFileName << " successfully created!\n";
        delete myDocument;
    }
    catch (string& e)
    {
        return false;
    }
    return true;
}

bool ReadXmlFile(string& szFileName, Map &map)
{
    try
    {
        TiXmlDocument *myDocument = new TiXmlDocument(szFileName.c_str());
        myDocument->LoadFile();
        TiXmlElement *RootElement = myDocument->RootElement();
        cout << "root value: " << RootElement->Value() << endl;
        if (string(RootElement->Value()) != "Map") {
            cout << "not a Map file!\n";
            return false;
        }
        map.areas.clear();
        for (TiXmlElement *aera_element = RootElement->FirstChildElement();
             aera_element != NULL; aera_element = aera_element->NextSiblingElement()) {
            if (string(aera_element->Value()) != "Aera") {
                cout << "not a Aera!\n";
                continue;
            }
            int id = -1;
            double T[6] = {0,};
            for (TiXmlAttribute* aera_attribute=aera_element->FirstAttribute();
                 aera_attribute != NULL; aera_attribute=aera_attribute->Next()) {
                if (string(aera_attribute->Name()) == "id") {
                    stringstream ss;
                    ss << string(aera_attribute->Value());
                    ss >> id;
                }
                if (string(aera_attribute->Name()) == "T") {
                    stringstream ss;
                    ss << string(aera_attribute->Value());
                    for (int i=0; i<6; i++) {
                        ss >> T[i];
                    }
                }
            }
            if (id == -1) {
                cout << "no id\n";
                continue;
            }
            Aera aera(id);
            cout << "aera " << id << endl;
            aera.setT(T);
            for (TiXmlElement *point_element = aera_element->FirstChildElement();
                 point_element != NULL; point_element = point_element->NextSiblingElement()) {
                if (string(point_element->Value()) != "_Point") {
                    cout << "not a _Point!\n";
                    continue;
                }
                int x=-1, y=-1;
                for (TiXmlAttribute* point_attribute=point_element->FirstAttribute();
                     point_attribute != NULL; point_attribute=point_attribute->Next()) {
                    if (string(point_attribute->Name()) == "x") {
                        stringstream ss;
                        ss << string(point_attribute->Value());
                        ss >> x;
                    } else if (string(point_attribute->Name()) == "y") {
                        stringstream ss;
                        ss << string(point_attribute->Value());
                        ss >> y;
                    }
                }
                if (x == -1 || y == -1) {
                    cout << "no x or y\n";
                    continue;
                }
                aera.points.push_back(_Point(x, y));
            }
            map.areas.push_back(aera);
        }
        cout << "loading map finished\n";
        delete myDocument;
    }
    catch (string& e)
    {
        return false;
    }
    return true;
}
