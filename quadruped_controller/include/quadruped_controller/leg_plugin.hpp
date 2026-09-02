// Copyright 2026 Jakub Delicat
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

#ifndef QUADRUPED_CONTROLLER__LEG_PLUGIN_HPP_
#define QUADRUPED_CONTROLLER__LEG_PLUGIN_HPP_

#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "quadruped_controller/leg_interface.hpp"

namespace quadruped_controller
{

/**
 * @brief Klasa bazowa dla wszystkich pluginów nóg
 *
 * Dostarcza wspólną funkcjonalność dla wszystkich implementacji nóg:
 * - Zarządzanie stanami stawów (JointState)
 * - Inicjalizacja nazw stawów
 * - Ustawianie pozycji z JointState
 * - Metody pomocnicze
 *
 * Klasy pochodne muszą zaimplementować:
 * - forward_kinematics()
 * - inverse_kinematics()
 * - kinematics()
 * - get_joints_directions()
 * - update_passive_joints()
 */
class LegPlugin : public LegInterface
{
public:
  LegPlugin() = default;
  ~LegPlugin() override = default;

  // ============================================
  // Implementacja wspólnych metod z LegInterface
  // ============================================

  /**
   * @brief Inicjalizuje nogę z podaną nazwą
   * @param name Nazwa nogi (np. "front_left", "front_right", itp.)
   */
  void initialize(const std::string & name);

  /**
   * @brief Ustawia pozycje stawów z wiadomości JointState
   * @param msg Wiadomość JointState
   */
  void set_positions_from_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) override;

  /**
   * @brief Pobiera pasywne stawy kolanowe
   * @return Para stawów (forth_, fifth_)
   */
  std::pair<JointState, JointState> get_passive_knee_joints() const override;

  /**
   * @brief Pobiera wszystkie stawy (5)
   * @return Tablica 5 stawów
   */
  std::array<JointState, 5> get_joints_states() const override;

  /**
   * @brief Pobiera aktywne stawy (3)
   * @return Tablica 3 stawów
   */
  std::array<JointState, 3> get_active_joint_states() override;

  /**
   * @brief Ustawia stany stawów
   * @param q Pozycje stawów
   * @param v Prędkości stawów
   * @param tau Momenty stawów
   */
  void set_joints_states(
    const Eigen::Vector3d & q, const Eigen::Vector3d & v = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & tau = Eigen::Vector3d::Zero()) override;

  /**
   * @brief Pobiera odległość do efektora
   * @return Odległość
   */
  double get_distance_to_effector() const override { return distance_to_effector_; }

  /**
   * @brief Pobiera nazwę nogi
   * @return Nazwa nogi
   */
  std::string get_name() const override { return name_; }

  /**
   * @brief Ustawia parametry nogi
   * @param params Wektor parametrów
   */
  void set_parameters(const std::vector<double> & params) override;

  // ============================================
  // Metody czysto wirtualne - muszą być zaimplementowane
  // przez klasy pochodne
  // ============================================

  virtual Eigen::Vector3d forward_kinematics(const Eigen::Vector3d & q) override = 0;
  virtual Eigen::Vector3d forward_kinematics() override = 0;
  virtual Eigen::Vector3d inverse_kinematics(const Eigen::Vector3d & x) override = 0;
  virtual Eigen::Matrix4d kinematics(const Eigen::Vector3d & q) override = 0;
  virtual Eigen::Vector3d get_joints_directions() const override = 0;

protected:
  std::string name_;                           ///< Nazwa nogi
  double third_joint_gear_correction_ = M_PI;  ///< Korekta przekładni

  JointState first_;   ///< Pierwszy staw (aktywny)
  JointState second_;  ///< Drugi staw (aktywny)
  JointState third_;   ///< Trzeci staw (aktywny)
  JointState forth_;   ///< Czwarty staw (pasywny)
  JointState fifth_;   ///< Piąty staw (pasywny)

  double distance_to_effector_ = 0.0;  ///< Odległość do efektora

  /**
   * @brief Aktualizuje pasywne stawy na podstawie pozycji trzeciego stawu
   * @param q3 Pozycja trzeciego stawu
   */
  virtual void update_passive_joints(double q3) = 0;
};

}  // namespace quadruped_controller

#endif  // QUADRUPED_CONTROLLER__LEG_PLUGIN_HPP_
