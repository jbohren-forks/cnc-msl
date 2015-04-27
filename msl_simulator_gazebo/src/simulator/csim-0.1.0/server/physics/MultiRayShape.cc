#include "XMLConfig.hh"
#include "MultiRayShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
MultiRayShape::MultiRayShape(Geom *parent) : Shape(parent)
{
  this->type = Shape::MULTIRAY;


  Param::Begin(&this->parameters);
  this->rayCountP = new ParamT<int>("rayCount",0,1);
  this->rangeCountP = new ParamT<int>("rangeCount",0,1);
  this->minAngleP = new ParamT<Angle>("minAngle",DTOR(-90),1);
  this->maxAngleP = new ParamT<Angle>("maxAngle",DTOR(-90),1);
  this->minRangeP = new ParamT<double>("minRange",0,1);
  this->maxRangeP = new ParamT<double>("maxRange",0,1);
  this->resRangeP = new ParamT<double>("resRange",0.1,1);
  this->originP = new ParamT<Vector3>("origin", Vector3(0,0,0), 0);

  // for block rays, vertical setting
  this->verticalRayCountP = new ParamT<int>("verticalRayCount", 1, 0);
  this->verticalRangeCountP = new ParamT<int>("verticalRangeCount", 1, 0);
  this->verticalMinAngleP = new ParamT<Angle>("verticalMinAngle", DTOR(0), 0);
  this->verticalMaxAngleP = new ParamT<Angle>("verticalMaxAngle", DTOR(0), 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
MultiRayShape::~MultiRayShape()
{

  std::vector< RayShape* >::iterator iter;

  for (iter=this->rays.begin(); iter!=this->rays.end(); iter++)
  {
    delete *iter;
  }
  this->rays.clear();

  delete this->rayCountP;
  delete this->rangeCountP;
  delete this->minAngleP;
  delete this->maxAngleP;
  delete this->minRangeP;
  delete this->maxRangeP;
  delete this->resRangeP;
  delete this->originP;

  delete this->verticalRayCountP;
  delete this->verticalRangeCountP;
  delete this->verticalMinAngleP;
  delete this->verticalMaxAngleP;

}

////////////////////////////////////////////////////////////////////////////////
// Load a multi-ray shape from xml file
void MultiRayShape::Load()
{
  this->rayCountP->Load();
  this->rangeCountP->Load();
  this->minAngleP->Load();
  this->maxAngleP->Load();
  this->minRangeP->Load();
  this->maxRangeP->Load();
  this->resRangeP->Load();
  this->originP->Load();
  this->verticalRayCountP->Load();
  this->verticalRangeCountP->Load();
  this->verticalMinAngleP->Load();
  this->verticalMaxAngleP->Load();
//}

////////////////////////////////////////////////////////////////////////////////
/*void MultiRayShape::Load(unsigned int vertRayCount, unsigned int rayCount,
            Vector3 origin, double minRange, double maxRange,
            Angle minVertAngle, Angle maxVertAngle,
            Angle minAngle, Angle maxAngle )
{
*/
  Vector3 start, end, axis;

  Angle yawAngle, pitchAngle; 
  Angle pDiff = **this->verticalMaxAngleP - **this->verticalMinAngleP;
  Angle yDiff = **this->maxAngleP - **this->minAngleP;

  //this->maxRange = maxRange;

  // Create and array of ray geoms
  for (int j = 0; j < **this->verticalRayCountP; j++)
  {
    for (int i = 0; i < **this->rayCountP; i++)
    {
      yawAngle = (**this->rayCountP == 1)? 0 : 
        i * yDiff.GetAsRadian() / (**this->rayCountP - 1) + 
        (**this->minAngleP).GetAsRadian();

      pitchAngle = (**this->verticalRayCountP == 1)? 0 :  
        j * pDiff.GetAsRadian() / (**this->verticalRayCountP - 1) + 
        (**this->verticalMinAngleP).GetAsRadian();

      axis.Set(cos(pitchAngle.GetAsRadian()) * 
          cos(yawAngle.GetAsRadian()), 
          sin(yawAngle.GetAsRadian()), 
          sin(pitchAngle.GetAsRadian())*
          cos(yawAngle.GetAsRadian()));

      start = (axis * **this->minRangeP) + **this->originP;
      end = (axis * **this->maxRangeP) + **this->originP;

      this->AddRay(start,end);
    }
  }

}

//////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void MultiRayShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << "  " << *(this->minAngleP) << "\n";
  stream << prefix << "  " << *(this->maxAngleP) << "\n";
  stream << prefix << "  " << *(this->minRangeP) << "\n";
  stream << prefix << "  " << *(this->maxRangeP) << "\n";
  stream << prefix << "  " << *(this->resRangeP) << "\n";
  stream << prefix << "  " << *(this->originP) << "\n";
  stream << prefix << "  " << *(this->rayCountP) << "\n";
  stream << prefix << "  " << *(this->rangeCountP) << "\n";
  stream << prefix << "  " << *(this->verticalRayCountP) << "\n";
  stream << prefix << "  " << *(this->verticalRangeCountP) << "\n";
  stream << prefix << "  " << *(this->verticalMinAngleP) << "\n";
  stream << prefix << "  " << *(this->verticalMaxAngleP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Get detected range for a ray.
/// \returns Returns DBL_MAX for no detection.
double MultiRayShape::GetRange(int index)
{
  if (index < 0 || index >= (int)this->rays.size())
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  return this->rays[index]->GetLength();
}

////////////////////////////////////////////////////////////////////////////////
/// Get detected retro (intensity) value for a ray.
double MultiRayShape::GetRetro(int index)
{
  if (index < 0 || index >= (int)this->rays.size())
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  return this->rays[index]->GetRetro();
}

////////////////////////////////////////////////////////////////////////////////
/// Get detected fiducial value for a ray.
int MultiRayShape::GetFiducial(int index)
{
  if (index < 0 || index >= (int)this->rays.size())
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  return this->rays[index]->GetFiducial();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the geom
void MultiRayShape::Update()
{
  std::vector<RayShape*>::iterator iter;
  Vector3 a, b;
  int i = 1;

  // Reset the ray lengths and mark the geoms as dirty (so they get
  // redrawn)
  for (iter = this->rays.begin(); 
      iter != this->rays.end(); iter++, i++)
  {
    (*iter)->SetLength( **this->maxRangeP );
    (*iter)->SetRetro( 0.0 );
    (*iter)->SetFiducial( -1 );

    // Get the global points of the line
    (*iter)->Update();
  }

  this->UpdateRays();

}

////////////////////////////////////////////////////////////////////////////////
/// Add a ray to the geom
void MultiRayShape::AddRay(const Vector3 &start, const Vector3 &end )
{

}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum angle
Angle MultiRayShape::GetMinAngle() const
{
  return **this->minAngleP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the maximum angle
Angle MultiRayShape::GetMaxAngle() const
{
  return **this->maxAngleP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum range
double MultiRayShape::GetMinRange() const
{
  return **this->minRangeP;
}

//////////////////////////////////////////////////////////////////////////////
///  Get the maximum range
double MultiRayShape::GetMaxRange() const
{
  return **this->maxRangeP;
}

//////////////////////////////////////////////////////////////////////////////
///  Get the range resolution
double MultiRayShape::GetResRange() const
{
  return **this->resRangeP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the ray count
int MultiRayShape::GetRayCount() const
{
  return **this->rayCountP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the range count
int MultiRayShape::GetRangeCount() const
{
  return **this->rangeCountP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical scan line count
int MultiRayShape::GetVerticalRayCount() const
{
  return **this->verticalRayCountP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical scan line count
int MultiRayShape::GetVerticalRangeCount() const
{
  return **this->verticalRangeCountP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical min angle
Angle MultiRayShape::GetVerticalMinAngle() const
{
  return **this->verticalMinAngleP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical max angle
Angle MultiRayShape::GetVerticalMaxAngle() const
{
  return **this->verticalMaxAngleP;
}

