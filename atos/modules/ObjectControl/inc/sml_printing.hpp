#include <iostream>
#include <functional>

// Code for dumping statemachine to text
template <class T>
static void dump_transition(std::ostream& out) noexcept {
auto src_state = std::string{boost::sml::aux::string<typename T::src_state>{}.c_str()};
auto dst_state = std::string{boost::sml::aux::string<typename T::dst_state>{}.c_str()};
if (dst_state == "X") {
    dst_state = "[*]";
}

if (T::initial) {
    out << "[*] --> " << src_state << "\n";
}

const auto has_event = !boost::sml::aux::is_same<typename T::event, boost::sml::anonymous>::value;
const auto has_guard = !boost::sml::aux::is_same<typename T::guard, boost::sml::front::always>::value;
const auto has_action = !boost::sml::aux::is_same<typename T::action, boost::sml::front::none>::value;

const auto is_entry = boost::sml::aux::is_same<typename T::event, boost::sml::back::on_entry<boost::sml::_, boost::sml::_>>::value;
const auto is_exit = boost::sml::aux::is_same<typename T::event, boost::sml::back::on_exit<boost::sml::_, boost::sml::_>>::value;

// entry / exit entry
if (is_entry || is_exit) {
    out << src_state;
} else {  // state to state transition
    out << src_state << " --> " << dst_state;
}

if (has_event || has_guard || has_action) {
    out << " :";
}

if (has_event) {
    // handle 'on_entry' and 'on_exit' per plant-uml syntax
    auto event = std::string(boost::sml::aux::get_type_name<typename T::event>());
    if (is_entry) {
    event = "entry";
    } else if (is_exit) {
    event = "exit";
    }
    out << " " << event;
}

if (has_guard) {
    out << " [" << boost::sml::aux::get_type_name<typename T::guard>() << "]";
}

if (has_action) {
    out << " / " << boost::sml::aux::get_type_name<typename T::action>();
}

out << "\n";
}

template <template <class...> class T, class... Ts>
static void dump_transitions(const T<Ts...>&, std::ostream& out) noexcept {
int _[]{0, (dump_transition<Ts>(out), 0)...};
(void)_;
}

template <class SM>
static void dump(const SM&, std::ostream& out) noexcept {
out << "@startuml\n\n";
dump_transitions(typename SM::transitions{}, out);
out << "\n@enduml\n";
}