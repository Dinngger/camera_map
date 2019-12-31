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
    _Point(const _Point &p) {
        this->x = p.x;
        this->y = p.y;
    }
};

class Aera
{
private:
    char T_string[100];
public:
    int id;
    double T[8];
    vector<_Point> points;
    Aera(int id) {
        this->id = id;
    }
    char* getT() {
        sprintf(T_string, "%f %f %f %f %f %f %f %f", T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7]);
        return T_string;
    }
    int setT(double T[]) {
        for (int i=0; i<8; i++) {
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
        int x = (T[0] * p.x + T[1] * p.y + T[2]) / (T[6] * p.x + T[7] * p.y + 1);
        int y = (T[3] * p.x + T[4] * p.y + T[5]) / (T[6] * p.x + T[7] * p.y + 1);
        return _Point(x, y);
    }
};

class Map
{
public:
    vector<Aera> aeras;
    bool draw_point(int p_cam[], _Point &p_map);
};

bool Map::draw_point(int p_cam[], _Point &p_map) {
    _Point mid((p_cam[0] + p_cam[2]) / 2, p_cam[3]);
    for (auto aera : aeras) {
        if (aera.inAera(mid)) {
            p_map = aera.transform(mid);
            return true;
        }
    }
    return false;
}

bool CreateXmlFile(string& szFileName, Map map)
{
    try
    {
        TiXmlDocument *myDocument = new TiXmlDocument();
        TiXmlElement *RootElement = new TiXmlElement("Map");
        myDocument->LinkEndChild(RootElement);
        for (auto aera : map.aeras) {
            TiXmlElement *aera_element = new TiXmlElement("Aera");
            RootElement->LinkEndChild(aera_element);
            aera_element->SetAttribute("id", aera.id);
            aera_element->SetAttribute("T", aera.getT());
            for (auto point : aera.points) {
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
        map.aeras.clear();
        for (TiXmlElement *aera_element = RootElement->FirstChildElement();
             aera_element != NULL; aera_element = aera_element->NextSiblingElement()) {
            if (string(aera_element->Value()) != "Aera") {
                cout << "not a Aera!\n";
                continue;
            }
            int id = -1;
            double T[8] = {0,};
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
                    for (int i=0; i<8; i++) {
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
                if (string(point_element->Value()) != "Point") {
                    cout << "not a Point!\n";
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
            map.aeras.push_back(aera);
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
