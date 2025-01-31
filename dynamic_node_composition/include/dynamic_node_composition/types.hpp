// Copyright 2024 Simon Sagmeister
#include <stdexcept>
#include <string>
namespace tam::dynamic_compositon
{
struct ComponentDescription
{
  std::string package_name;
  std::string component_name;
  ComponentDescription(std::string pkg_name, std::string cmpnt_name)
  : package_name(pkg_name), component_name(cmpnt_name)
  {
  }
};
namespace exceptions
{
struct ComponentNotFound : public std::runtime_error
{
  explicit ComponentNotFound(ComponentDescription const & cmpnt)
  : std::runtime_error(
      std::string("DynamicComposition | Component not found! | Package: " + cmpnt.package_name) +
      " -> Component: " + cmpnt.component_name)
  {
  }
};
}  // namespace exceptions
}  // namespace tam::dynamic_compositon
