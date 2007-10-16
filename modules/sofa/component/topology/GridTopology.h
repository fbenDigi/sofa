/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#ifndef SOFA_COMPONENT_TOPOLOGY_GRIDTOPOLOGY_H
#define SOFA_COMPONENT_TOPOLOGY_GRIDTOPOLOGY_H

#include <sofa/component/topology/MeshTopology.h>

namespace sofa
{

namespace component
{

namespace topology
{

using namespace sofa::defaulttype;

class GridTopology : public MeshTopology
{
public:
    GridTopology();

    GridTopology(int nx, int ny, int nz);

    void setSize(int nx, int ny, int nz);

    void parse(core::objectmodel::BaseObjectDescription* arg)
    {
        this->MeshTopology::parse(arg);

        const char* nx = arg->getAttribute("nx","1");
        const char* ny = arg->getAttribute("ny","1");
        const char* nz = arg->getAttribute("nz","1");
        n.setValue(Vec<3,int>(atoi(nx),atoi(ny),atoi(nz)));

        this->setSize();
    }

    int getNx() const { return n.getValue()[0]; }
    int getNy() const { return n.getValue()[1]; }
    int getNz() const { return n.getValue()[2]; }

    void setNx(int n_) { (*n.beginEdit())[0] = n_; setSize(); }
    void setNy(int n_) { (*n.beginEdit())[1] = n_; setSize(); }
    void setNz(int n_) { (*n.beginEdit())[2] = n_; setSize(); }

    //int getNbPoints() const { return n.getValue()[0]*n.getValue()[1]*n.getValue()[2]; }

    int getNbCubes() { return (n.getValue()[0]-1)*(n.getValue()[1]-1)*(n.getValue()[2]-1); }

    int getNbQuads()
    {
        if (n.getValue()[2] == 1)
            return (n.getValue()[0]-1)*(n.getValue()[1]-1);
        else if (n.getValue()[1] == 1)
            return (n.getValue()[0]-1)*(n.getValue()[2]-1);
        else
            return (n.getValue()[1]-1)*(n.getValue()[2]-1);
    }

    Cube getCube(int i);
    Cube getCube(int x, int y, int z);

    Quad getQuad(int i);
    Quad getQuad(int x, int y, int z);

    int point(int x, int y, int z) const { return x+n.getValue()[0]*(y+n.getValue()[1]*z); }
    int cube(int x, int y, int z) const { return x+(n.getValue()[0]-1)*(y+(n.getValue()[1]-1)*z); }

protected:
    DataField< Vec<3, int> > n;

    virtual void setSize();
    void updateLines();
    void updateQuads();
    void updateCubes();
};

} // namespace topology

} // namespace component

} // namespace sofa

#endif
