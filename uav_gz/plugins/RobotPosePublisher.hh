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

#ifndef UAV_GZ__PLUGINS__ROBOTPOSEPUBLISHER_HH_
#define UAV_GZ__PLUGINS__ROBOTPOSEPUBLISHER_HH_

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

class RobotPosePublisherPrivate;

/// \brief Sistema publicador de pose para modelos de robot.
/// Adjuntar a una entidad modelo para publicar su pose y la de sus
/// entidades hijo.
///
/// ## Parámetros SDF
///
/// - `<topic>`: Tópico para publicar poses.
///   Valor por defecto: "/model/<nombre_modelo>/pose"
/// - `<use_pose_vector_msg>`: Si es true, publica gz::msgs::Pose_V.
///   Por defecto: false.
/// - `<update_frequency>`: Frecuencia de publicación en Hz.
///   Valor negativo = publicar lo más rápido posible. Por defecto: -1.
/// - `<publish_link_poses>`: Si es true, publica la pose de todos los links.
///   Por defecto: false.
/// - `<publish_canonical_link_only>`: Si es true, publica solo el link
///   canónico. Tiene prioridad sobre publish_link_poses. Por defecto: false.
/// - `<enable_statistics>`: Imprime estadísticas cada 500 publicaciones.
///   Por defecto: false.
class RobotPosePublisher
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate,
      public ISystemReset
{
  /// \brief Constructor
  public: RobotPosePublisher();

  /// \brief Destructor
  public: ~RobotPosePublisher() override = default;

  // Documentación heredada
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override;

  // Documentación heredada
  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override;

  // Documentación heredada — maneja el reset de la simulación
  public: void Reset(const UpdateInfo &_info,
                     EntityComponentManager &_ecm) override;

  /// \brief Puntero a los datos privados (patrón Pimpl)
  private: std::unique_ptr<RobotPosePublisherPrivate> dataPtr;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // UAV_GZ__PLUGINS__ROBOTPOSEPUBLISHER_HH_
