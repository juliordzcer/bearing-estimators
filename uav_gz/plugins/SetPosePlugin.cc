// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0

#include "SetPosePlugin.hh"

#include <cmath>

#include <mutex>
#include <optional>
#include <string>

#include <gz/msgs/pose.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Datos privados para SetPosePlugin
class gz::sim::systems::SetPosePluginPrivate
{
  /// \brief Nodo de transporte
  public: transport::Node node;

  /// \brief El modelo al que está adjunto este plugin
  public: Model model{kNullEntity};

  /// \brief Tópico al que se suscribe el plugin
  public: std::string topic;

  /// \brief Nueva pose pendiente de aplicar (protegida por mutex)
  public: std::optional<math::Pose3d> newPose;

  /// \brief Mutex para acceso seguro a newPose desde el callback
  public: std::mutex poseMutex;

  /// \brief Si se deben imprimir logs de debug
  public: bool enableDebugLog{false};

  /// \brief Si es true, el plugin se desuscribe tras aplicar la
  /// primera pose. Ideal para spawn inicial: libera el thread del
  /// transport durante toda la simulación posterior.
  public: bool applyOnce{false};

  /// \brief Flag de inicialización válida
  public: bool initialized{false};

  /// \brief Valida que los valores de una pose no sean NaN ni Inf
  /// \param[in] _msg El mensaje de pose a validar
  /// \return true si la pose es válida
  public: bool ValidatePoseMsg(const msgs::Pose &_msg) const
  {
    const auto &pos = _msg.position();
    const auto &ori = _msg.orientation();

    // Verificar NaN e Inf en posición
    if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z()) ||
        std::isinf(pos.x()) || std::isinf(pos.y()) || std::isinf(pos.z()))
    {
      gzwarn << "SetPosePlugin: posición inválida (NaN o Inf) recibida en ["
             << this->topic << "]. Ignorando mensaje." << std::endl;
      return false;
    }

    // Verificar NaN e Inf en orientación
    if (std::isnan(ori.w()) || std::isnan(ori.x()) ||
        std::isnan(ori.y()) || std::isnan(ori.z()) ||
        std::isinf(ori.w()) || std::isinf(ori.x()) ||
        std::isinf(ori.y()) || std::isinf(ori.z()))
    {
      gzwarn << "SetPosePlugin: orientación inválida (NaN o Inf) recibida en ["
             << this->topic << "]. Ignorando mensaje." << std::endl;
      return false;
    }

    // Verificar que el cuaternión no sea de norma cero (degenerado)
    const double norm = std::sqrt(ori.w()*ori.w() + ori.x()*ori.x() +
                                  ori.y()*ori.y() + ori.z()*ori.z());
    if (norm < 1e-6)
    {
      gzwarn << "SetPosePlugin: cuaternión de norma ~0 recibido en ["
             << this->topic << "]. Ignorando mensaje." << std::endl;
      return false;
    }

    return true;
  }
};

//////////////////////////////////////////////////
SetPosePlugin::SetPosePlugin()
  : dataPtr(std::make_unique<SetPosePluginPrivate>())
{
}

//////////////////////////////////////////////////
void SetPosePlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "SetPosePlugin debe estar adjunto a una entidad modelo válida. "
          << "Falló al inicializar." << std::endl;
    return;
  }

  // Parsear log de debug (opcional)
  this->dataPtr->enableDebugLog =
      _sdf->Get<bool>("enable_debug_log", false).first;

  // Parsear apply_once
  this->dataPtr->applyOnce =
      _sdf->Get<bool>("apply_once", false).first;

  // Parsear tópico — usa valor por defecto si no se especifica
  if (_sdf->HasElement("topic"))
  {
    this->dataPtr->topic = _sdf->Get<std::string>("topic");
    if (this->dataPtr->topic.empty())
    {
      gzwarn << "SetPosePlugin: elemento <topic> vacío. "
             << "Usando tópico por defecto." << std::endl;
    }
  }

  if (this->dataPtr->topic.empty())
  {
    this->dataPtr->topic =
        "/model/" + this->dataPtr->model.Name(_ecm) + "/set_pose";
  }

  // Suscribirse al tópico.
  // gz-transport13 requiere std::function explícito o puntero a método
  // miembro — los lambdas con captura no se deducen automáticamente.
  std::function<void(const msgs::Pose &)> cb =
    [this](const msgs::Pose &_msg)
    {
      if (!this->dataPtr->ValidatePoseMsg(_msg))
        return;

      std::lock_guard<std::mutex> lock(this->dataPtr->poseMutex);
      this->dataPtr->newPose = math::Pose3d(
        math::Vector3d(_msg.position().x(),
                       _msg.position().y(),
                       _msg.position().z()),
        math::Quaterniond(_msg.orientation().w(),
                          _msg.orientation().x(),
                          _msg.orientation().y(),
                          _msg.orientation().z())
      );

      if (this->dataPtr->enableDebugLog)
      {
        gzdbg << "SetPosePlugin [" << this->dataPtr->topic
              << "]: pose recibida -> pos("
              << _msg.position().x() << ", "
              << _msg.position().y() << ", "
              << _msg.position().z() << ")" << std::endl;
      }
    };

  if (!this->dataPtr->node.Subscribe(this->dataPtr->topic, cb))
  {
    gzerr << "SetPosePlugin: error al suscribirse al tópico ["
          << this->dataPtr->topic << "]." << std::endl;
    return;
  }

  this->dataPtr->initialized = true;

  gzmsg << "SetPosePlugin configurado:" << std::endl
        << "  - Tópico: " << this->dataPtr->topic << std::endl
        << "  - Modo: "
        << (this->dataPtr->applyOnce
              ? "apply_once (se desuscribe tras la primera pose)"
              : "continuo")
        << std::endl
        << "  - Debug log: "
        << (this->dataPtr->enableDebugLog ? "activado" : "desactivado")
        << std::endl;
}

//////////////////////////////////////////////////
void SetPosePlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("SetPosePlugin::PreUpdate");

  // No operar si no está correctamente inicializado
  if (!this->dataPtr->initialized)
    return;

  // No aplicar poses mientras la simulación está pausada
  if (_info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->poseMutex);
  if (this->dataPtr->newPose.has_value())
  {
    this->dataPtr->model.SetWorldPoseCmd(_ecm, *this->dataPtr->newPose);

    if (this->dataPtr->enableDebugLog)
    {
      gzdbg << "SetPosePlugin: pose aplicada -> "
            << this->dataPtr->newPose->Pos() << std::endl;
    }

    this->dataPtr->newPose.reset();

    // Si apply_once=true, desuscribirse inmediatamente tras aplicar
    // la pose. El nodo deja de recibir callbacks y libera el thread
    // del transport para el resto de la simulación.
    if (this->dataPtr->applyOnce)
    {
      this->dataPtr->node.Unsubscribe(this->dataPtr->topic);
      gzdbg << "SetPosePlugin [" << this->dataPtr->topic
            << "]: apply_once=true, desuscrito." << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void SetPosePlugin::Reset(const UpdateInfo &/*_info*/,
    EntityComponentManager &/*_ecm*/)
{
  // Al resetear la simulación, descartar cualquier pose pendiente
  std::lock_guard<std::mutex> lock(this->dataPtr->poseMutex);
  this->dataPtr->newPose.reset();

  gzdbg << "SetPosePlugin: reset — pose pendiente descartada." << std::endl;
}

// Registrar el plugin
GZ_ADD_PLUGIN(gz::sim::systems::SetPosePlugin,
              gz::sim::System,
              gz::sim::systems::SetPosePlugin::ISystemConfigure,
              gz::sim::systems::SetPosePlugin::ISystemPreUpdate,
              gz::sim::systems::SetPosePlugin::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::SetPosePlugin,
                    "gz::sim::systems::SetPosePlugin")