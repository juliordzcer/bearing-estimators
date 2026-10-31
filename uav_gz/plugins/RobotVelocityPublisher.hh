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

#ifndef UAV_GZ__PLUGINS__ROBOTVELOCITYPUBLISHER_HH_
#define UAV_GZ__PLUGINS__ROBOTVELOCITYPUBLISHER_HH_

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

class RobotVelocityPublisherPrivate;

/// \brief Publica la velocidad (lineal y angular) del link canónico del modelo
/// expresada en el marco de referencia del propio link (body frame).
///
/// ## Parámetros SDF
///
/// - `<topic>`: Tópico para publicar mensajes gz::msgs::Twist.
///   Valor por defecto: "/model/<nombre_modelo>/velocity"
/// - `<update_frequency>`: Frecuencia de publicación en Hz.
///   Valor negativo = publicar lo más rápido posible. Por defecto: -1.
/// - `<frame_id>`: Frame ID para el header del mensaje.
///   Por defecto: nombre del link canónico.
/// - `<enable_statistics>`: Si es true, imprime estadísticas cada 1000 msgs.
///   Por defecto: false.
class RobotVelocityPublisher
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate,
      public ISystemReset
{
  /// \brief Constructor
  public: RobotVelocityPublisher();

  /// \brief Destructor
  public: ~RobotVelocityPublisher() override = default;

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
  private: std::unique_ptr<RobotVelocityPublisherPrivate> dataPtr;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // UAV_GZ__PLUGINS__ROBOTVELOCITYPUBLISHER_HH_
