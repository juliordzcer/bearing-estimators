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

#include "RobotVelocityPublisher.hh"

#include <cstdint>
#include <string>

#include <gz/msgs/twist.pb.h>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Conversions.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Datos privados para RobotVelocityPublisher
class gz::sim::systems::RobotVelocityPublisherPrivate
{
  /// \brief Nodo de transporte
  public: transport::Node node;

  /// \brief Publicador de mensajes Twist
  public: transport::Node::Publisher twistPub;

  /// \brief Frecuencia de actualización en Hz (-1 = sin límite)
  public: double updateFrequency{-1};

  /// \brief Período de actualización calculado a partir de updateFrequency
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Último tiempo en que se publicó un mensaje
  public: std::chrono::steady_clock::duration lastVelPubTime{0};

  /// \brief Mensaje Twist reutilizable
  public: msgs::Twist twistMsg;

  /// \brief Modelo al que está adjunto este plugin
  public: Model model{kNullEntity};

  /// \brief Entidad del link del cual se lee la velocidad
  public: Entity linkEntity{kNullEntity};

  /// \brief Tópico de publicación
  public: std::string topic;

  /// \brief Frame ID para el header del mensaje
  public: std::string frameId;

  /// \brief Habilitar estadísticas de publicación
  public: bool enableStatistics{false};

  /// \brief Contador de mensajes publicados
  public: uint64_t publishCount{0};

  /// \brief Indica si la inicialización fue exitosa
  public: bool initialized{false};

  /// \brief Indica si se usó un link de fallback (no canónico)
  public: bool usedFallbackLink{false};

  /// \brief Indica si ya se mostró el error de link no encontrado en PostUpdate
  public: bool linkErrorLogged{false};
};

//////////////////////////////////////////////////
RobotVelocityPublisher::RobotVelocityPublisher()
  : dataPtr(std::make_unique<RobotVelocityPublisherPrivate>())
{
}

//////////////////////////////////////////////////
void RobotVelocityPublisher::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "RobotVelocityPublisher debe estar adjunto a una entidad modelo. "
          << "Falló al inicializar." << std::endl;
    return;
  }

  // Parsear frecuencia de actualización
  double freq = _sdf->Get<double>("update_frequency", -1.0).first;
  if (freq > 0.0)
  {
    if (freq > 1000.0)
    {
      gzwarn << "RobotVelocityPublisher: frecuencia muy alta ("
             << freq << " Hz). Limitando a 1000 Hz." << std::endl;
      freq = 1000.0;
    }
    this->dataPtr->updateFrequency = freq;
    std::chrono::duration<double> period{1.0 / freq};
    this->dataPtr->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  } else if (freq == 0.0)
  {
    gzwarn << "RobotVelocityPublisher: frecuencia 0 Hz no válida. "
           << "Desactivando control de frecuencia." << std::endl;
  }

  // Parsear tópico
  if (_sdf->HasElement("topic"))
  {
    this->dataPtr->topic = _sdf->Get<std::string>("topic");
  }
  if (this->dataPtr->topic.empty())
  {
    this->dataPtr->topic =
        "/model/" + this->dataPtr->model.Name(_ecm) + "/velocity";
  }

  // Parsear frame_id
  this->dataPtr->frameId = _sdf->Get<std::string>("frame_id", "").first;

  // Parsear estadísticas
  this->dataPtr->enableStatistics =
      _sdf->Get<bool>("enable_statistics", false).first;

  // Crear publicador
  this->dataPtr->twistPub =
      this->dataPtr->node.Advertise<msgs::Twist>(this->dataPtr->topic);

  // Buscar el link canónico
  this->dataPtr->linkEntity = this->dataPtr->model.CanonicalLink(_ecm);

  // Fallback: primer link disponible si no hay link canónico
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    auto links = _ecm.ChildrenByComponents(
        this->dataPtr->model.Entity(), components::Link());
    if (!links.empty())
    {
      this->dataPtr->linkEntity = links.front();
      this->dataPtr->usedFallbackLink = true;
      gzwarn << "RobotVelocityPublisher: link canónico no encontrado para ["
             << this->dataPtr->model.Name(_ecm)
             << "]. Usando primer link como fallback." << std::endl;
    } else {
      gzerr << "RobotVelocityPublisher: no se encontró ningún link para ["
            << this->dataPtr->model.Name(_ecm) << "]. "
            << "Falló al inicializar." << std::endl;
      return;
    }
  }

  // Habilitar cómputo de velocidades en el ECM (body frame)
  if (!_ecm.Component<components::LinearVelocity>(this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity,
                         components::LinearVelocity());
  }
  if (!_ecm.Component<components::AngularVelocity>(this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity,
                         components::AngularVelocity());
  }

  this->dataPtr->initialized = true;

  gzmsg << "RobotVelocityPublisher configurado:" << std::endl
        << "  - Tópico: " << this->dataPtr->topic << std::endl
        << "  - Frecuencia: "
        << (this->dataPtr->updateFrequency > 0
              ? std::to_string(this->dataPtr->updateFrequency) + " Hz"
              : "sin límite")
        << std::endl
        << "  - Frame ID: "
        << (this->dataPtr->frameId.empty()
              ? "(nombre del link)"
              : this->dataPtr->frameId)
        << (this->dataPtr->usedFallbackLink ? " [link fallback]" : "")
        << std::endl
        << "  - Estadísticas: "
        << (this->dataPtr->enableStatistics ? "activadas" : "desactivadas")
        << std::endl;
}

//////////////////////////////////////////////////
void RobotVelocityPublisher::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("RobotVelocityPublisher::PostUpdate");

  if (!this->dataPtr->initialized)
    return;

  if (_info.paused)
    return;

  // Manejar reset del tiempo de simulación
  if (_info.simTime < this->dataPtr->lastVelPubTime)
  {
    this->dataPtr->lastVelPubTime = _info.simTime;
    return;
  }

  // Control de frecuencia
  if (this->dataPtr->updateFrequency > 0)
  {
    auto diff = _info.simTime - this->dataPtr->lastVelPubTime;
    if (diff < this->dataPtr->updatePeriod)
      return;
  }

  // Verificar que el link sigue siendo válido
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    if (!this->dataPtr->linkErrorLogged)
    {
      gzerr << "RobotVelocityPublisher: link no encontrado para ["
            << this->dataPtr->model.Name(_ecm)
            << "]. No se publicará velocidad." << std::endl;
      this->dataPtr->linkErrorLogged = true;
    }
    return;
  }

  // Obtener componentes de velocidad
  auto linVelComp =
      _ecm.Component<components::LinearVelocity>(this->dataPtr->linkEntity);
  auto angVelComp =
      _ecm.Component<components::AngularVelocity>(this->dataPtr->linkEntity);

  // Esperar a que el sistema de física los inicialice
  if (!linVelComp || !angVelComp)
    return;

  const math::Vector3d &linVel = linVelComp->Data();
  const math::Vector3d &angVel = angVelComp->Data();

  // Construir el mensaje Twist
  this->dataPtr->twistMsg.Clear();
  auto *header = this->dataPtr->twistMsg.mutable_header();
  header->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));

  auto *frame = header->add_data();
  frame->set_key("frame_id");

  if (!this->dataPtr->frameId.empty())
  {
    frame->add_value(this->dataPtr->frameId);
  } else {
    auto linkName =
        _ecm.Component<components::Name>(this->dataPtr->linkEntity);
    frame->add_value(linkName ? linkName->Data() : "unknown");
  }

  msgs::Set(this->dataPtr->twistMsg.mutable_linear(), linVel);
  msgs::Set(this->dataPtr->twistMsg.mutable_angular(), angVel);

  this->dataPtr->twistPub.Publish(this->dataPtr->twistMsg);
  this->dataPtr->publishCount++;
  this->dataPtr->lastVelPubTime = _info.simTime;

  if (this->dataPtr->enableStatistics &&
      this->dataPtr->publishCount % 1000 == 0)
  {
    gzmsg << "RobotVelocityPublisher: " << this->dataPtr->publishCount
          << " mensajes publicados en [" << this->dataPtr->topic << "]"
          << std::endl;
  }
}

//////////////////////////////////////////////////
void RobotVelocityPublisher::Reset(const UpdateInfo &/*_info*/,
    EntityComponentManager &/*_ecm*/)
{
  this->dataPtr->lastVelPubTime =
      std::chrono::steady_clock::duration::zero();
  this->dataPtr->publishCount = 0;
  this->dataPtr->linkErrorLogged = false;

  gzdbg << "RobotVelocityPublisher: estado interno reseteado." << std::endl;
}

// Registrar el plugin
GZ_ADD_PLUGIN(gz::sim::systems::RobotVelocityPublisher,
              gz::sim::System,
              gz::sim::systems::RobotVelocityPublisher::ISystemConfigure,
              gz::sim::systems::RobotVelocityPublisher::ISystemPostUpdate,
              gz::sim::systems::RobotVelocityPublisher::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::RobotVelocityPublisher,
                    "gz::sim::systems::RobotVelocityPublisher")
