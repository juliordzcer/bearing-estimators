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

#include "RobotPosePublisher.hh"

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Conversions.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Datos privados para RobotPosePublisher
class gz::sim::systems::RobotPosePublisherPrivate
{
  /// \brief Inicializa el mapa de entidades a publicar.
  /// Debe llamarse una vez, tras la primera iteración válida.
  public: void InitializeEntitiesToPublish(const EntityComponentManager &_ecm);

  /// \brief Llena el vector _poses con las poses actuales de las entidades
  public: void FillPoses(
      const EntityComponentManager &_ecm,
      std::vector<std::pair<Entity, math::Pose3d>> &_poses) const;

  /// \brief Publica las poses usando el publicador dado
  public: void PublishPoses(
      const std::vector<std::pair<Entity, math::Pose3d>> &_poses,
      const msgs::Time &_stampMsg,
      transport::Node::Publisher &_publisher);

  /// \brief Nodo de transporte
  public: transport::Node node;

  /// \brief Publicador de poses
  public: transport::Node::Publisher posePub;

  /// \brief Si publicar poses de todos los links del modelo
  public: bool publishLinkPoses{false};

  /// \brief Si publicar solo el link canónico (tiene prioridad)
  public: bool publishCanonicalLinkOnly{false};

  /// \brief Frecuencia de actualización (-1 = sin límite)
  public: double updateFrequency{-1};

  /// \brief Último tiempo de publicación
  public: std::chrono::steady_clock::duration lastPosePubTime{0};

  /// \brief Período de actualización calculado
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Mapa: entidad -> (frame_id, child_frame_id)
  public: std::unordered_map<Entity,
              std::pair<std::string, std::string>> entitiesToPublish;

  /// \brief Buffer reutilizable de poses
  public: std::vector<std::pair<Entity, math::Pose3d>> poses;

  /// \brief Mensaje Pose reutilizable (para modo no-vectorial)
  public: msgs::Pose poseMsg;

  /// \brief Mensaje Pose_V reutilizable (para modo vectorial)
  public: msgs::Pose_V poseVMsg;

  /// \brief Si publicar usando Pose_V en lugar de Pose individual
  public: bool usePoseV{false};

  /// \brief Si imprimir estadísticas de publicación
  public: bool enableStatistics{false};

  /// \brief Contador de publicaciones
  public: uint64_t publishCount{0};

  /// \brief Indica si ya se inicializaron las entidades a publicar
  public: bool initialized{false};

  /// \brief Modelo al que está adjunto este plugin
  public: Model model{kNullEntity};

  /// \brief Tópico de publicación
  public: std::string topic;

  /// \brief Entidad del link canónico
  public: Entity canonicalLinkEntity{kNullEntity};
};

//////////////////////////////////////////////////
RobotPosePublisher::RobotPosePublisher()
  : dataPtr(std::make_unique<RobotPosePublisherPrivate>())
{
}

//////////////////////////////////////////////////
void RobotPosePublisher::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "RobotPosePublisher debe estar adjunto a una entidad modelo. "
          << "Falló al inicializar." << std::endl;
    return;
  }

  // Parsear parámetros de configuración
  this->dataPtr->usePoseV =
      _sdf->Get<bool>("use_pose_vector_msg", false).first;

  this->dataPtr->publishLinkPoses =
      _sdf->Get<bool>("publish_link_poses", false).first;

  this->dataPtr->publishCanonicalLinkOnly =
      _sdf->Get<bool>("publish_canonical_link_only", false).first;

  this->dataPtr->enableStatistics =
      _sdf->Get<bool>("enable_statistics", false).first;

  // Parsear frecuencia con validación
  double freq = _sdf->Get<double>("update_frequency", -1.0).first;
  if (freq > 0.0)
  {
    if (freq > 1000.0)
    {
      gzwarn << "RobotPosePublisher: frecuencia muy alta ("
             << freq << " Hz). Limitando a 1000 Hz." << std::endl;
      freq = 1000.0;
    }
    this->dataPtr->updateFrequency = freq;
    std::chrono::duration<double> period{1.0 / freq};
    this->dataPtr->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  } else if (freq == 0.0)
  {
    gzwarn << "RobotPosePublisher: frecuencia 0 Hz no válida. "
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
        "/model/" + this->dataPtr->model.Name(_ecm) + "/pose";
  }

  // Buscar link canónico
  this->dataPtr->canonicalLinkEntity =
      this->dataPtr->model.CanonicalLink(_ecm);
  if (this->dataPtr->canonicalLinkEntity == kNullEntity)
  {
    gzwarn << "RobotPosePublisher: no se encontró link canónico para ["
           << this->dataPtr->model.Name(_ecm) << "]." << std::endl;
  }

  // Crear publicador según tipo de mensaje
  if (this->dataPtr->usePoseV)
  {
    this->dataPtr->posePub =
        this->dataPtr->node.Advertise<msgs::Pose_V>(this->dataPtr->topic);
  } else {
    this->dataPtr->posePub =
        this->dataPtr->node.Advertise<msgs::Pose>(this->dataPtr->topic);
  }

  gzmsg << "RobotPosePublisher configurado:" << std::endl
        << "  - Tópico: " << this->dataPtr->topic << std::endl
        << "  - Frecuencia: "
        << (this->dataPtr->updateFrequency > 0
              ? std::to_string(this->dataPtr->updateFrequency) + " Hz"
              : "sin límite")
        << std::endl
        << "  - Tipo de mensaje: "
        << (this->dataPtr->usePoseV ? "Pose_V" : "Pose") << std::endl
        << "  - Publicar links: "
        << (this->dataPtr->publishLinkPoses ? "sí" : "no") << std::endl
        << "  - Solo link canónico: "
        << (this->dataPtr->publishCanonicalLinkOnly ? "sí" : "no") << std::endl
        << "  - Estadísticas: "
        << (this->dataPtr->enableStatistics ? "activadas" : "desactivadas")
        << std::endl;
}

//////////////////////////////////////////////////
void RobotPosePublisher::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("RobotPosePublisher::PostUpdate");

  // Manejar reset de tiempo de simulación
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    this->dataPtr->lastPosePubTime = _info.simTime;
    return;
  }

  if (_info.paused)
    return;

  // Control de frecuencia
  if (this->dataPtr->updateFrequency > 0)
  {
    auto diff = _info.simTime - this->dataPtr->lastPosePubTime;
    if (diff > std::chrono::steady_clock::duration::zero() &&
        diff < this->dataPtr->updatePeriod)
    {
      return;
    }
  }

  // Inicializar entidades la primera vez (lazy init: el ECM ya está completo)
  if (!this->dataPtr->initialized)
  {
    this->dataPtr->InitializeEntitiesToPublish(_ecm);
    this->dataPtr->initialized = true;

    if (this->dataPtr->entitiesToPublish.empty())
    {
      gzwarn << "RobotPosePublisher: no se encontraron entidades para publicar "
             << "en el modelo [" << this->dataPtr->model.Name(_ecm)
             << "]." << std::endl;
    }
  }

  this->dataPtr->poses.clear();
  this->dataPtr->FillPoses(_ecm, this->dataPtr->poses);

  if (!this->dataPtr->poses.empty())
  {
    this->dataPtr->PublishPoses(
        this->dataPtr->poses,
        convert<msgs::Time>(_info.simTime),
        this->dataPtr->posePub);

    this->dataPtr->publishCount++;

    if (this->dataPtr->enableStatistics &&
        this->dataPtr->publishCount % 500 == 0)
    {
      gzmsg << "RobotPosePublisher: " << this->dataPtr->publishCount
            << " publicaciones en [" << this->dataPtr->topic << "]"
            << std::endl;
    }
  }

  this->dataPtr->lastPosePubTime = _info.simTime;
}

//////////////////////////////////////////////////
void RobotPosePublisher::Reset(const UpdateInfo &/*_info*/,
    EntityComponentManager &/*_ecm*/)
{
  this->dataPtr->lastPosePubTime =
      std::chrono::steady_clock::duration::zero();
  this->dataPtr->publishCount = 0;
  // Re-inicializar entidades en el próximo PostUpdate para reflejar
  // cualquier cambio en el modelo durante el reset
  this->dataPtr->initialized = false;
  this->dataPtr->entitiesToPublish.clear();

  gzdbg << "RobotPosePublisher: estado interno reseteado." << std::endl;
}

//////////////////////////////////////////////////
void RobotPosePublisherPrivate::InitializeEntitiesToPublish(
    const EntityComponentManager &_ecm)
{
  this->entitiesToPublish.clear();
  const std::string modelName = this->model.Name(_ecm);

  // Prioridad 1: solo link canónico
  if (this->publishCanonicalLinkOnly)
  {
    if (this->canonicalLinkEntity != kNullEntity)
    {
      auto linkName =
          _ecm.Component<components::Name>(this->canonicalLinkEntity);
      if (linkName)
      {
        this->entitiesToPublish[this->canonicalLinkEntity] =
            std::make_pair("world", modelName + "/" + linkName->Data());
      } else {
        gzwarn << "RobotPosePublisher: link canónico sin componente Name."
               << std::endl;
      }
    } else {
      gzwarn << "RobotPosePublisher: publish_canonical_link_only=true pero "
             << "no se encontró link canónico en [" << modelName << "]."
             << std::endl;
    }
    return;
  }

  // Prioridad 2: todos los links del modelo
  if (this->publishLinkPoses)
  {
    auto links = _ecm.ChildrenByComponents(
        this->model.Entity(), components::Link());

    if (links.empty())
    {
      gzwarn << "RobotPosePublisher: publish_link_poses=true pero no se "
             << "encontraron links en [" << modelName << "]. "
             << "Publicando pose del modelo." << std::endl;
      // Caer al caso default (publicar pose del modelo)
    } else {
      for (const auto &link : links)
      {
        auto linkName = _ecm.Component<components::Name>(link);
        if (linkName)
        {
          this->entitiesToPublish[link] =
              std::make_pair("world", modelName + "/" + linkName->Data());
        }
      }
      return;
    }
  }

  // Default: publicar la pose del propio modelo
  this->entitiesToPublish[this->model.Entity()] =
      std::make_pair("world", modelName);
}

//////////////////////////////////////////////////
void RobotPosePublisherPrivate::FillPoses(
    const EntityComponentManager &_ecm,
    std::vector<std::pair<Entity, math::Pose3d>> &_poses) const
{
  GZ_PROFILE("RobotPosePublisher::FillPoses");

  for (const auto &[entity, _] : this->entitiesToPublish)
  {
    auto pose = _ecm.Component<components::Pose>(entity);
    if (!pose)
      continue;
    _poses.emplace_back(entity, pose->Data());
  }
}

//////////////////////////////////////////////////
void RobotPosePublisherPrivate::PublishPoses(
    const std::vector<std::pair<Entity, math::Pose3d>> &_poses,
    const msgs::Time &_stampMsg,
    transport::Node::Publisher &_publisher)
{
  GZ_PROFILE("RobotPosePublisher::PublishPoses");

  if (this->usePoseV)
    this->poseVMsg.Clear();

  for (const auto &[entity, pose] : _poses)
  {
    auto entityIt = this->entitiesToPublish.find(entity);
    if (entityIt == this->entitiesToPublish.end())
      continue;

    msgs::Pose *msg = nullptr;
    if (this->usePoseV)
    {
      msg = this->poseVMsg.add_pose();
    } else {
      this->poseMsg.Clear();
      msg = &this->poseMsg;
    }

    // Rellenar header con timestamp y frame info
    auto *header = msg->mutable_header();
    header->mutable_stamp()->CopyFrom(_stampMsg);

    const std::string &frameId    = entityIt->second.first;
    const std::string &childFrame = entityIt->second.second;

    auto *frameData = header->add_data();
    frameData->set_key("frame_id");
    frameData->add_value(frameId);

    auto *childFrameData = header->add_data();
    childFrameData->set_key("child_frame_id");
    childFrameData->add_value(childFrame);

    msg->set_name(childFrame);
    msgs::Set(msg, pose);

    // Publicar individualmente si no es Pose_V
    if (!this->usePoseV)
      _publisher.Publish(this->poseMsg);
  }

  // Publicar el vector completo al final
  if (this->usePoseV)
    _publisher.Publish(this->poseVMsg);
}

// Registrar el plugin
GZ_ADD_PLUGIN(gz::sim::systems::RobotPosePublisher,
              gz::sim::System,
              gz::sim::systems::RobotPosePublisher::ISystemConfigure,
              gz::sim::systems::RobotPosePublisher::ISystemPostUpdate,
              gz::sim::systems::RobotPosePublisher::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::RobotPosePublisher,
                    "gz::sim::systems::RobotPosePublisher")
