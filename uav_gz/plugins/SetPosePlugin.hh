// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UAV_GZ__PLUGINS__SETPOSEPLUGIN_HH_
#define UAV_GZ__PLUGINS__SETPOSEPLUGIN_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{

class SetPosePluginPrivate;

/// \brief Plugin para establecer la pose de un modelo mediante mensajes
/// publicados en un tópico de transporte.
///
/// ## Parámetros SDF
///
/// - `<topic>`: Tópico para recibir mensajes gz::msgs::Pose.
///   Valor por defecto: "/model/<nombre_modelo>/set_pose"
/// - `<enable_debug_log>`: Si es true, imprime logs de debug al recibir
///   y aplicar poses. Por defecto: false.
class SetPosePlugin
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemReset
{
  /// \brief Constructor
  public: SetPosePlugin();

  /// \brief Destructor
  public: ~SetPosePlugin() override = default;

  // Documentación heredada
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override;

  // Documentación heredada
  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override;

  // Documentación heredada — maneja el reset de la simulación
  public: void Reset(const UpdateInfo &_info,
                     EntityComponentManager &_ecm) override;

  /// \brief Puntero a los datos privados (patrón Pimpl)
  private: std::unique_ptr<SetPosePluginPrivate> dataPtr;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // UAV_GZ__PLUGINS__SETPOSEPLUGIN_HH_
