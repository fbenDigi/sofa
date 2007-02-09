#include "ObjectFactory.h"

namespace sofa
{

namespace core
{

ObjectFactory::ClassEntry* ObjectFactory::getEntry(std::string classname)
{
    ClassEntry*& p = registry[classname];
    if (p==NULL)
    {
        p = new ClassEntry;
        p->className = classname;
    }
    return p;
}

bool ObjectFactory::addAlias(std::string name, std::string result, bool force)
{
    std::map<std::string, ClassEntry*>::iterator it = registry.find(result);
    if (it == registry.end())
    {
        std::cerr << "ERROR: ObjectFactory: cannot create alias "<<name<<" to unknown class " << result << ".\n";
        return false;
    }
    ClassEntry* entry = it->second;
    ClassEntry*& p = registry[name];
    if (p!=NULL && !force)
    {
        std::cerr << "ERROR: ObjectFactory: cannot create alias "<<name<<" as a class with this name already exists.\n";
        return false;
    }
    else
    {
        if (p!=NULL)
        {
            p->aliases.erase(name);
        }
        p = entry;
        entry->aliases.insert(name);
        return true;
    }
}

objectmodel::BaseObject* ObjectFactory::createObject(objectmodel::BaseContext* context, objectmodel::BaseObjectDescription* arg)
{
    objectmodel::BaseObject* object = NULL;
    std::vector<Creator*> creators;
    std::string classname = arg->getAttribute( "type", "");
    std::string templatename = arg->getAttribute( "template", "");
    std::map<std::string, ClassEntry*>::iterator it = registry.find(classname);
    if (it != registry.end())
    {
        ClassEntry* entry = it->second;
        std::map<std::string, Creator*>::iterator it2 = entry->creators.find(templatename);
        if (it2 != entry->creators.end())
        {
            Creator* c = it2->second;
            if (c->canCreate(context, arg))
                creators.push_back(c);
        }
        else
        {
            for (it2 = entry->creators.begin(); it2 != entry->creators.end(); ++it2)
            {
                Creator* c = it2->second;
                if (c->canCreate(context, arg))
                    creators.push_back(c);
            }
        }
    }
    if (creators.empty())
    {
        std::cerr<<"ERROR: ObjectFactory: Object type "<<classname<<"<"<<templatename<<"> creation failed."<<std::endl;
    }
    else
    {
        if (creators.size()>1)
        {
            std::cerr<<"WARNING: ObjectFactory: Several possibilities found for type "<<classname<<"<"<<templatename<<">."<<std::endl;
        }
        object = creators[0]->createInstance(context, arg);
    }
    return object;
}

ObjectFactory* ObjectFactory::getInstance()
{
    static ObjectFactory instance;
    return &instance;
}

void ObjectFactory::dump(std::ostream& out)
{
    for (std::map<std::string, ClassEntry*>::iterator it = registry.begin(), itend = registry.end(); it != itend; ++it)
    {
        ClassEntry* entry = it->second;
        if (entry->className != it->first) continue;
        out << "class " << entry->className <<" :\n";
        if (!entry->aliases.empty())
        {
            out << "  aliases :";
            for (std::set<std::string>::iterator it = entry->aliases.begin(), itend = entry->aliases.end(); it != itend; ++it)
                out << " " << *it;
            out << "\n";
        }
        if (!entry->baseClasses.empty())
        {
            out << "  base classes :";
            for (std::set<std::string>::iterator it = entry->baseClasses.begin(), itend = entry->baseClasses.end(); it != itend; ++it)
                out << " " << *it;
            out << "\n";
        }
        if (!entry->description.empty())
            out << entry->description;
        if (!entry->authors.empty())
            out << "  authors : " << entry->authors << "\n";
        if (!entry->license.empty())
            out << "  license : " << entry->license << "\n";
        for (std::map<std::string, Creator*>::iterator itc = entry->creators.begin(), itcend = entry->creators.end(); itc != itcend; ++itc)
        {
            out << "  template instance : " << itc->first << "\n";
        }
    }
}

static std::string xmlencode(const std::string& str)
{
    std::string res;
    for (unsigned int i=0; i<str.length(); ++i)
    {
        switch(str[i])
        {
        case '<': res += "&lt;"; break;
        case '>': res += "&gt;"; break;
        case '&': res += "&amp;"; break;
        default:  res += str[i];
        }
    }
    return res;
}

void ObjectFactory::dumpXML(std::ostream& out)
{
    for (std::map<std::string, ClassEntry*>::iterator it = registry.begin(), itend = registry.end(); it != itend; ++it)
    {
        ClassEntry* entry = it->second;
        if (entry->className != it->first) continue;
        out << "<class name=\"" << xmlencode(entry->className) <<"\">\n";
        for (std::set<std::string>::iterator it = entry->aliases.begin(), itend = entry->aliases.end(); it != itend; ++it)
            out << "<alias>" << xmlencode(*it) << "</alias>\n";
        for (std::set<std::string>::iterator it = entry->baseClasses.begin(), itend = entry->baseClasses.end(); it != itend; ++it)
            out << "<base>" << *it << "</base>\n";
        if (!entry->description.empty())
            out << "<description>"<<entry->description<<"</description>\n";
        if (!entry->authors.empty())
            out << "<authors>"<<entry->authors<<"</authors>\n";
        if (!entry->license.empty())
            out << "<license>"<<entry->license<<"</license>\n";
        for (std::map<std::string, Creator*>::iterator itc = entry->creators.begin(), itcend = entry->creators.end(); itc != itcend; ++itc)
        {
            out << "<creator";
            if (!itc->first.empty()) out << " template=\"" << xmlencode(itc->first) << "\"";
            out << "/>\n";
        }
        out << "</class>\n";
    }
}

void ObjectFactory::dumpHTML(std::ostream& out)
{
    out << "<ul>\n";
    for (std::map<std::string, ClassEntry*>::iterator it = registry.begin(), itend = registry.end(); it != itend; ++it)
    {
        ClassEntry* entry = it->second;
        if (entry->className != it->first) continue;
        out << "<li><b>" << xmlencode(entry->className) <<"</b>\n";
        if (!entry->description.empty())
            out << "<br/>"<<entry->description<<"\n";
        out << "<ul>\n";
        if (!entry->aliases.empty())
        {
            out << "<li>Aliases:<i>";
            for (std::set<std::string>::iterator it = entry->aliases.begin(), itend = entry->aliases.end(); it != itend; ++it)
                out << " " << xmlencode(*it);
            out << "</i></li>\n";
        }
        if (!entry->baseClasses.empty())
        {
            out << "<li>Base Classes:<b>";
            for (std::set<std::string>::iterator it = entry->baseClasses.begin(), itend = entry->baseClasses.end(); it != itend; ++it)
                out << " " << xmlencode(*it);
            out << "</b></li>\n";
        }
        if (!entry->authors.empty())
            out << "<li>Authors: <i>"<<entry->authors<<"</i></li>\n";
        if (!entry->license.empty())
            out << "<li>License: <i>"<<entry->license<<"</i></li>\n";
        if (entry->creators.size()>2 || (entry->creators.size()==1 && !entry->creators.begin()->first.empty()))
        {
            out << "<li>Template instances:<i>";
            for (std::map<std::string, Creator*>::iterator itc = entry->creators.begin(), itcend = entry->creators.end(); itc != itcend; ++itc)
            {
                out << " " << xmlencode(itc->first);
            }
            out << "</i></li>\n";
        }
        out << "</ul>\n";
        out << "</li>\n";
    }
    out << "</ul>\n";
}

RegisterObject::RegisterObject(const std::string& description)
{
    if (!description.empty())
        addDescription(description);
}

RegisterObject RegisterObject::addAlias(std::string val)
{
    entry.aliases.insert(val);
    return *this;
}

RegisterObject RegisterObject::addDescription(std::string val)
{
    //std::cout << "ObjectFactory: add description "<<val<<std::endl;
    val += '\n';
    entry.description += val;
    return *this;
}

RegisterObject RegisterObject::addAuthor(std::string val)
{
    //std::cout << "ObjectFactory: add author "<<val<<std::endl;
    val += ' ';
    entry.authors += val;
    return *this;
}

RegisterObject RegisterObject::addLicense(std::string val)
{
    //std::cout << "ObjectFactory: add license "<<val<<std::endl;
    entry.license += val;
    return *this;
}

RegisterObject RegisterObject::addCreator(std::string classname, std::string templatename, ObjectFactory::Creator* creator)
{
    //std::cout << "ObjectFactory: add creator "<<classname<<" with template "<<templatename<<std::endl;
    if (!entry.className.empty() && entry.className != classname)
    {
        std::cerr << "ERROR: ObjectFactory: all templated class should have the same base classname ("<<entry.className<<"!="<<classname<<")\n";
    }
    else if (entry.creators.find(templatename) != entry.creators.end())
    {
        std::cerr << "ERROR: ObjectFactory: class "<<classname<<"<"<<templatename<<"> already registered\n";
    }
    else
    {
        entry.className = classname;
        entry.creators.insert(std::make_pair(templatename, creator));
    }
    return *this;
}

RegisterObject::operator int()
{
    if (entry.className.empty())
    {
        return 0;
    }
    else
    {
        //std::cout << "ObjectFactory: commit"<<std::endl;
        ObjectFactory::ClassEntry* reg = ObjectFactory::getInstance()->getEntry(entry.className);
        reg->description += entry.description;
        reg->authors += entry.authors;
        reg->license += entry.license;
        for (std::map<std::string, ObjectFactory::Creator*>::iterator itc = entry.creators.begin(), itcend = entry.creators.end(); itc != itcend; ++itc)
        {
            if (reg->creators.find(itc->first) != reg->creators.end())
            {
                std::cerr << "ERROR: ObjectFactory: class "<<entry.className<<"<"<<itc->first<<"> already registered\n";
            }
            else
            {
                reg->creators.insert(*itc);
            }
        }
        for (std::set<std::string>::iterator it = entry.aliases.begin(), itend = entry.aliases.end(); it != itend; ++it)
        {
            if (reg->aliases.find(*it) == reg->aliases.end())
            {
                ObjectFactory::getInstance()->addAlias(*it,entry.className);
            }
        }
        for (std::set<std::string>::iterator it = entry.baseClasses.begin(), itend = entry.baseClasses.end(); it != itend; ++it)
        {
            if (reg->baseClasses.find(*it) == reg->baseClasses.end())
            {
                reg->baseClasses.insert(*it);
            }
        }
        //std::cout << "ObjectFactory: commit end"<<std::endl;
        return 1;
    }
}

} // namespace core

} // namespace sofa
