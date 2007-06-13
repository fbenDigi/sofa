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
#include <sofa/helper/io/MassSpringLoader.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/defaulttype/Vec.h>

#include <stdio.h>
#include <iostream>
#include <vector>

namespace sofa
{

namespace helper
{

namespace io
{

using namespace sofa::defaulttype;

static void skipToEOL(FILE* f)
{
    int	ch;
    while ((ch = fgetc(f)) != EOF && ch != '\n');
}

bool MassSpringLoader::load(const char *filename)
{
    std::string fname = filename;
    if (!sofa::helper::system::DataRepository.findFile(fname)) return false;

    char cmd[64];
    FILE* file;

    if ((file = fopen(fname.c_str(), "r")) == NULL)
    {
        std::cout << "ERROR: cannot read file '" << filename << "'. Exiting..." << std::endl;
        return false;
    }
    std::cout << "Loading model '" << filename << "'" << std::endl;
    int totalNumMasses=0;
    int totalNumSprings=0;
    // Check first line

    //if (fgets(cmd, 7, file) == NULL || !strcmp(cmd,"Xsp 4.0"))
    if (fgets(cmd, 7, file) == NULL)
    {
        fclose(file);
        return false;
    }

    // paul--------------------------
    float version = 0.0;
    sscanf(cmd, "Xsp %f", &version);

    bool  vector_spring = false;
    if (version == 3.0) vector_spring = false;
    else if (version == 4.0) vector_spring = true;
    else
    {
        fclose(file);
        return false;
    }
    // paul----------------------------

    skipToEOL(file);

    // then find out number of masses and springs
    if (fscanf(file, "%s", cmd) != EOF && !strcmp(cmd,"numm"))
    {
        fscanf(file, "%d", &totalNumMasses);
        setNumMasses(totalNumMasses);
    }
    if (fscanf(file, "%s", cmd) != EOF && !strcmp(cmd,"nums"))
    {
        fscanf(file, "%d", &totalNumSprings);
        setNumSprings(totalNumSprings);
    }

    std::cout << "Model contains "<< totalNumMasses <<" masses and "<< totalNumSprings <<" springs"<<std::endl;

    std::vector<Vec3d> masses;
    if (totalNumMasses>0)
        masses.reserve(totalNumMasses);

    while (fscanf(file, "%s", cmd) != EOF)
    {
        if (!strcmp(cmd,"mass"))
        {
            int index;
            char location;
            double px,py,pz,vx,vy,vz,mass=0.0,elastic=0.0;
            bool fixed=false;
            fscanf(file, "%d %c %lf %lf %lf %lf %lf %lf %lf %lf\n",
                    &index, &location,
                    &px, &py, &pz, &vx, &vy, &vz,
                    &mass, &elastic);
            bool surface = (location == 's');

            if (mass < 0)
            {
                // fixed point initialization
                mass = -mass;
                fixed = true;
            }
            addMass(px,py,pz,vx,vy,vz,mass,elastic,fixed,surface);
            masses.push_back(Vec3d(px,py,pz));
        }
        else if (!strcmp(cmd,"lspg"))	// linear springs connector
        {
            int	index;
            int m1,m2;
            double ks=0.0,kd=0.0,initpos=-1;
            // paul-------------------------------------
            double restx=0.0,resty=0.0,restz=0.0;
            if (vector_spring)
                fscanf(file, "%d %d %d %lf %lf %lf %lf %lf %lf\n",
                        &index,&m1,&m2,&ks,&kd,&initpos, &restx,&resty,&restz);
            else
                fscanf(file, "%d %d %d %lf %lf %lf\n",
                        &index,&m1,&m2,&ks,&kd,&initpos);
            --m1;
            --m2;
            if (!masses.empty() && ((unsigned int)m1>=masses.size() || (unsigned int)m2>=masses.size()))
            {
                std::cerr << "ERROR: incorrect mass indexes in spring "<<index<<" "<<m1+1<<" "<<m2+1<<std::endl;
            }
            else
            {
                if (initpos==-1 && !masses.empty())
                {
                    initpos = (masses[m1]-masses[m2]).norm();
                    ks/=initpos;
                    kd/=initpos;
                    //std::cout << "spring "<<m1<<" "<<m2<<" "<<ks<<" "<<kd<<" "<<initpos<<"\n";
                }

                //paul-----------------------------------------
                if (vector_spring)
                    addVectorSpring(m1,m2,ks,kd,initpos,restx,resty,restz);
                else
                    addSpring(m1,m2,ks,kd,initpos);
            }
        }
        else if (!strcmp(cmd,"grav"))
        {
            double gx,gy,gz;
            fscanf(file, "%lf %lf %lf\n", &gx, &gy, &gz);
            setGravity(gx,gy,gz);
        }
        else if (!strcmp(cmd,"visc"))
        {
            double viscosity;
            fscanf(file, "%lf\n", &viscosity);
            setViscosity(viscosity);
        }
        else if (!strcmp(cmd,"step"))
        {
            //fscanf(file, "%lf\n", &(MSparams.default_dt));
            skipToEOL(file);
        }
        else if (!strcmp(cmd,"frce"))
        {
            skipToEOL(file);
        }
        else if (cmd[0] == '#')	// it's a comment
        {
            skipToEOL(file);
        }
        else		// it's an unknown keyword
        {
            printf("%s: Unknown MassSpring keyword: %s\n", filename, cmd);
            skipToEOL(file);
        }
    }
    fclose(file);
    return true;
}

} // namespace io

} // namespace helper

} // namespace sofa

