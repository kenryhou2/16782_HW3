#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <unordered_set>
#include <string>
#include <memory> // For smart pointers
#include <chrono>
#include <cstring>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6
#ifndef ENVS_DIR
#define ENVS_DIR "../envs"
#endif
class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;
using std::unordered_set;
using std::vector;
using std::shared_ptr;

bool print_status = true;

class GroundedCondition
{
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(const string &predicate, const list<string> &arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth; // fixed
        for (const string &l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition &gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth; // fixed
        for (const string &l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const GroundedCondition &pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition &rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        temp += this->predicate;
        temp += "(";
        for (const string &l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition &lhs, const GroundedCondition &rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition &gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(const string &pred, const list<string> &args, const bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (const string &ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const Condition &cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition &rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (const string &l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition &lhs, const Condition &rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition &cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(const string &name, const list<string> &args,
           const unordered_set<Condition, ConditionHasher, ConditionComparator> &preconditions,
           const unordered_set<Condition, ConditionHasher, ConditionComparator> &effects)
    {
        this->name = name;
        for (const string &l : args)
        {
            this->args.push_back(l);
        }
        for (const Condition &pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (const Condition &pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action &rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream &operator<<(ostream &os, const Action &ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (const Condition &precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (const Condition &effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->get_name();
        temp += "(";
        for (const string &l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action &lhs, const Action &rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action &ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(const GroundedCondition &gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(const GroundedCondition &gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(const GroundedCondition &gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(const GroundedCondition &gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(const string &symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(const list<string> &symbols)
    {
        for (const string &l : symbols)
            this->symbols.insert(l);
    }
    void add_action(const Action &action)
    {
        this->actions.insert(action);
    }

    Action get_action(const string &name) const
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    friend ostream &operator<<(ostream &os, const Env &w)
    {
        os << "***** Environment *****" << endl
           << endl;
        os << "Symbols: ";
        for (const string &s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (const GroundedCondition &s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (const GroundedCondition &g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (const Action &g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }

    // add getters
    auto get_initial_conditions() const
    {
        return this->initial_conditions;
    }
    auto get_goal_conditions() const
    {
        return this->goal_conditions;
    }
    auto get_actions() const
    {
        return this->actions;
    }
};

class GroundedAction
{
    string name;
    list<string> arg_values;

public:
    GroundedAction(const string &name, const list<string> &arg_values)
    {
        this->name = name;
        for (const string &ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction &rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream &operator<<(ostream &os, const GroundedAction &gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->name;
        temp += "(";
        for (const string &l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env *create_env(char *filename)
{
    ifstream input_file(filename);
    Env *env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line.empty())
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str())); // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

// --- after forward declarations of the hasher/comparator structs ---
using SetofGCond = std::unordered_set<
    GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>;
using SetofCond  = std::unordered_set<
    Condition,        ConditionHasher,        ConditionComparator>;

struct Node {
    SetofGCond conditions;
    double g{};
    double h{};
    double f{};
    std::shared_ptr<Node> parent;
    Action action;

    Node(std::shared_ptr<Node> parent_,
         const SetofGCond& conditions_,
         double g_, double h_,
         const Action& action_)
        : conditions(conditions_),
          g(g_), h(h_), f(g_ + h_),
          parent(std::move(parent_)),
          action(action_) {}
};

// --------- combos / perms utilities ----------

/**
 * @brief Generate all r-combinations (unordered, no repetition) from a symbol list.
 *
 * This is a recursive helper that builds up one partial combination `combination`
 * and appends full-sized combinations to `combinations`.
 *
 * @param symbols       Universe of candidate symbols.
 * @param r             Target combination size.
 * @param combinations  Output accumulator: each entry is one r-combination.
 * @param combination   Scratch buffer holding the current partial combination.
 * @param start         Index in `symbols` from which to continue choosing.
 *
 * @complexity O(C(n,r)) states produced; per state O(1) push/pop (not counting copying when pushing).
 */
inline void generateCombinations(const std::vector<std::string>& symbols,
                                 int r,
                                 std::vector<std::vector<std::string>>& combinations,
                                 std::vector<std::string>& combination,
                                 int start) {
    if (static_cast<int>(combination.size()) == r) {
        combinations.push_back(combination);
        return;
    }
    for (int i = start; i < static_cast<int>(symbols.size()); ++i) {
        combination.push_back(symbols[i]);
        generateCombinations(symbols, r, combinations, combination, i + 1);
        combination.pop_back();
    }
}

/**
 * @brief Generate all permutations of a given combination (i.e., ordered arrangements).
 *
 * Starts from a sorted copy and iterates all lexicographic permutations,
 * pushing each permuted sequence into `permutations`.
 *
 * @param combination   One combination (unordered) of symbols.
 * @param permutations  Output accumulator: each entry is an ordered permutation.
 *
 * @complexity O(k * k!) for k = combination.size(); each next_permutation is O(k).
 */
inline void generatePermutations(const std::vector<std::string>& combination,
                                 std::vector<std::vector<std::string>>& permutations) {
    std::vector<std::string> perm = combination;
    std::sort(perm.begin(), perm.end());
    do { permutations.push_back(perm); } while (std::next_permutation(perm.begin(), perm.end()));
}

/**
 * @brief Generate all permutations of all r-combinations from `symbols`.
 *
 * First enumerates all r-combinations (unordered); for each combination,
 * enumerates all orderings. This corresponds to "permutations of size r without repetition".
 *
 * @param symbols  Universe of symbols to choose from.
 * @param numArgs  r = the arity (number of arguments) required.
 * @return std::vector<std::vector<std::string>>  All length-r ordered tuples drawn from symbols.
 *
 * @complexity O(C(n,r) * r!) sequences produced; storage proportional to result size.
 */
inline std::vector<std::vector<std::string>>
generateGroundings(const std::vector<std::string>& symbols, int numArgs) {
    std::vector<std::vector<std::string>> combinations;
    std::vector<std::string> combination;
    generateCombinations(symbols, numArgs, combinations, combination, 0);

    std::vector<std::vector<std::string>> permutations;
    for (const auto& comb : combinations) generatePermutations(comb, permutations);
    return permutations;
}

/**
 * @brief Cache all (permutations of combinations) grouped by arity across actions in an Env.
 *
 * For each distinct action arity in the environment, precompute all ordered
 * tuples of that length from `symbols`. Avoids recomputing Permutation and combos per action.
 *
 * @param env      The planning environment (provides the set of actions).
 * @param symbols  Universe of symbols available for grounding.
 * @return std::unordered_map<int, std::vector<std::vector<std::string>>>
 *         Map: arity -> list of ordered tuples (strings) of that length.
 *
 * @note If multiple actions share the same arity, the permutation and combination list is computed once.
 */
inline std::unordered_map<int, std::vector<std::vector<std::string>>>
generateGroundingMap(const Env* env, const std::vector<std::string>& symbols) {
    std::unordered_map<int, std::vector<std::vector<std::string>>> permutation_combo_map;
    for (const Action& action : env->get_actions()) {
        int numArgs = static_cast<int>(action.get_args().size());
        if (!permutation_combo_map.count(numArgs)) permutation_combo_map[numArgs] = generateGroundings(symbols, numArgs);
    }
    return permutation_combo_map;
}

// --------- small printers (optional) ----------

/**
 * @brief Print the argument names of an action to stdout.
 *
 * @param action  Action whose formal parameters (names/placeholders) to print.
 * @output        Writes to std::cout.
 */
inline void printActionArgs(const Action& action) {
    const std::list<std::string>& args = action.get_args();
    std::cout << "Arguments: ";
    for (const std::string& arg : args) std::cout << arg << " ";
    std::cout << std::endl;
}

/**
 * @brief Print preconditions of an action to stdout.
 *
 * @param action  Action whose preconditions to print (pretty-prints each Condition).
 * @output        Writes to std::cout.
 */
inline void printPreconditions(const Action& action) {
    const auto& preconditions = action.get_preconditions();
    std::cout << "Preconditions: ";
    for (const auto& precond : preconditions) std::cout << precond << " ";
    std::cout << std::endl;
}

/**
 * @brief Print a set of grounded conditions (a state) to stdout.
 *
 * @param conditions  The state as a set of GroundedCondition.
 * @output            Writes to std::cout.
 */
inline void printGConditions(const SetofGCond& conditions) {
    std::cout << "Ground conditions: ";
    for (const auto& condition : conditions) std::cout << condition << " ";
    std::cout << std::endl;
}

// --------- argument mapping helpers ----------

/**
 * @brief Build a mapping from old formal parameter names to new bound argument values.
 *
 * Typically used to specialize action schemas (formal args) to a specific symbol tuple.
 *
 * @param old_args  Formal parameter list (e.g., {"x","y"}).
 * @param new_args  Concrete values list of same length (e.g., {"A","B"}).
 * @return std::unordered_map<std::string,std::string>  Map old->new.
 * @throws std::runtime_error if the two lists differ in length.
 */
inline std::unordered_map<std::string, std::string>
create_arg_map(const std::list<std::string>& old_args,
               const std::list<std::string>& new_args) {
    if (old_args.size() != new_args.size())
        throw std::runtime_error("Number of arguments do not match!");

    std::unordered_map<std::string, std::string> arg_map;
    auto it_old = old_args.begin();
    auto it_new = new_args.begin();
    while (it_old != old_args.end() && it_new != new_args.end()) {
        arg_map[*it_old] = *it_new;
        ++it_old; ++it_new;
    }
    return arg_map;
}

/**
 * @brief Substitute action precondition arguments according to a given argument map.
 *
 * Creates a new set of preconditions where each formal parameter is replaced
 * by the corresponding concrete value from `new_args`.
 *
 * @param action    The action schema providing original preconditions and formals.
 * @param new_args  Concrete values bound to action's formals (same arity/order).
 * @return SetofCond  New preconditions (Condition) with substituted arguments.
 */
inline SetofCond update_preconditions_args(const Action& action,
                                           const std::list<std::string>& new_args) {
    auto arg_map = create_arg_map(action.get_args(), new_args);
    SetofCond updated_preconditions;

    for (const auto& precond : action.get_preconditions()) {
        std::list<std::string> precond_args = precond.get_args();
        for (auto& arg : precond_args) {
            auto it = arg_map.find(arg);             // avoid C++17 if-init
            if (it != arg_map.end()) arg = it->second;
        }
        updated_preconditions.insert(
            Condition(precond.get_predicate(), precond_args, precond.get_truth()));
    }
    return updated_preconditions;
}

/**
 * @brief Substitute action effect arguments according to a given argument map.
 *
 * Creates a new set of effects where each formal parameter is replaced
 * by the corresponding concrete value from `new_args`.
 *
 * @param action    The action schema providing original effects and formals.
 * @param new_args  Concrete values bound to action's formals (same arity/order).
 * @return SetofCond  New effects (Condition) with substituted arguments.
 */
inline SetofCond update_effects_args(const Action& action,
                                     const std::list<std::string>& new_args) {
    auto arg_map = create_arg_map(action.get_args(), new_args);
    SetofCond updated_effects;

    for (const auto& effect : action.get_effects()) {
        std::list<std::string> effect_args = effect.get_args();
        for (auto& arg : effect_args) {
            auto it = arg_map.find(arg);             // avoid C++17 if-init
            if (it != arg_map.end()) arg = it->second;
        }
        updated_effects.insert(
            Condition(effect.get_predicate(), effect_args, effect.get_truth()));
    }
    return updated_effects;
}

/**
 * @brief Specialize an action schema with a concrete tuple of symbols.
 *
 * Applies argument substitution to both preconditions and effects, and returns
 * a new Action with the same name and the bound argument list.
 *
 * @param action     Action schema to specialize.
 * @param perm_list  Concrete argument values (length must equal action arity).
 * @return Action    A new Action with substituted args/preconditions/effects.
 */
inline Action updateActionWithSymbols(const Action& action,
                                      const std::list<std::string>& perm_list) {
    SetofCond updated_preconditions = update_preconditions_args(action, perm_list);
    SetofCond updated_effects       = update_effects_args(action, perm_list);
    return Action(action.get_name(), perm_list, updated_preconditions, updated_effects);
}

// --------- state tests & transitions ----------

/**
 * @brief Check if all preconditions of an action are satisfied in a grounded state.
 *
 * Converts each (possibly negated) precondition into a GroundedCondition using the
 * already-substituted arguments, and tests membership in `conditions`.
 *
 * @param act         A fully-grounded action (arguments already specialized).
 * @param conditions  Current world state as a set of GroundedCondition facts.
 * @return true if every precondition is satisfied; false otherwise.
 */
inline bool ArePrecondSatisfied(const Action& act, const SetofGCond& conditions) {
    for (const Condition& precond : act.get_preconditions()) {
        GroundedCondition pregc(precond.get_predicate(), precond.get_args(), precond.get_truth());
        if (conditions.find(pregc) == conditions.end()) return false;
    }
    return true;
}

/**
 * @brief Apply a grounded action's effects to a state and return the successor state.
 *
 * Positive effects are inserted; negative effects (delete list) are erased.
 *
 * @param action      A fully-grounded action.
 * @param conditions  Current world state.
 * @return SetofGCond New state after applying effects.
 */
inline SetofGCond applyEffect(const Action& action, const SetofGCond& conditions) {
    SetofGCond newConditions = conditions;
    for (const Condition& effect : action.get_effects()) {
        if (effect.get_truth()) {
            newConditions.insert(GroundedCondition(effect.get_predicate(), effect.get_args()));
        } else {
            newConditions.erase (GroundedCondition(effect.get_predicate(), effect.get_args()));
        }
    }
    return newConditions;
}

/**
 * @brief Test whether all goal conditions are contained in the current state.
 *
 * @param conditions       Current world state.
 * @param goal_conditions  Target set of GroundedCondition facts.
 * @return true if every goal fact is present; false otherwise.
 */
inline bool IsGoalReached(const SetofGCond& conditions, const SetofGCond& goal_conditions) {
    for (const GroundedCondition& goal : goal_conditions) {
        if (conditions.find(goal) == conditions.end()) return false;
    }
    return true;
}

/**
 * @brief Check if a state is absent from the CLOSED set of expanded nodes.
 *
 * @param conditions  State to test (as a set of grounded facts).
 * @param closed      Set of already-expanded nodes.
 * @return true if no node in CLOSED has the same state's conditions; false otherwise.
 */
inline bool NotInClosedList(const SetofGCond& conditions,
                            const std::unordered_set<std::shared_ptr<Node>>& closed) {
    for (const std::shared_ptr<Node>& node : closed) {
        if (node->conditions == conditions) return false;
    }
    return true;
}

// --------- PQ helpers (templated to accept any comparator type) ----------

/**
 * @brief Linear scan to test membership of a state in the OPEN priority queue.
 *
 * Makes a copy of the priority queue to iterate without mutating the original.
 * This is templated to accept any comparator type used by the caller.
 *
 * @tparam PriorityQueue  A std::priority_queue<shared_ptr<Node>, ..., Comparator>.
 * @param conditions      State to test.
 * @param open            The OPEN list (priority queue), passed by const reference.
 * @return true if an element with identical state's conditions exists; false otherwise.
 *
 * @complexity O(|OPEN|) comparisons; requires copying the PQ (heap) once.
 */
template<class PriorityQueue>
inline bool IsInOpenList(const SetofGCond& conditions, const PriorityQueue& open) {
    auto open_copy = open; // copy to iterate
    while (!open_copy.empty()) {
        std::shared_ptr<Node> node = open_copy.top();
        open_copy.pop();
        if (node->conditions == conditions) return true;
    }
    return false;
}

/**
 * @brief Update a matching node in OPEN if a better (lower g/f) path is found.
 *
 * Iterates through a copied-out sequence (by popping everything), updates the
 * matching state if found, then pushes all items back into OPEN.
 *
 * @tparam PriorityQueue  A std::priority_queue<shared_ptr<Node>, ..., Comparator>.
 * @param newNode         Candidate node with same state's conditions and better cost.
 * @param open            The OPEN list to update (passed by non-const reference).
 *
 * @complexity O(|OPEN| log |OPEN|) because we pop then push all elements.
 */
template<class PriorityQueue>
inline void UpdateOpenList(const std::shared_ptr<Node>& newNode, PriorityQueue& open) {
    std::vector<std::shared_ptr<Node>> temp;
    while (!open.empty()) {
        std::shared_ptr<Node> node = open.top();
        open.pop();
        if (node->conditions == newNode->conditions) {
            if (newNode->g < node->g) {
                node->g = newNode->g;
                node->h = newNode->h;
                node->f = newNode->f;
                node->parent = newNode->parent;
                node->action = newNode->action;
            }
        }
        temp.push_back(node);
    }
    for (const auto& node : temp) open.push(node);
}

// --------- heuristic ----------

/**
 * @brief Compute search heuristic for a state, supporting multiple modes.
 *
 * Modes:
 *  - 0: Dijkstra (h=0)
 *  - 1: # of missing goal conditions
 *  - 2: Penalty for delete effects in the chosen action (empty-delete-list heuristic)
 *  - 3: (1) + (2) combined
 *
 * @param conditions       Current world state.
 * @param goal_conditions  Target set of GroundedCondition facts.
 * @param mode             Heuristic mode selector (0..3).
 * @param action           The action being considered (used in delete-effect penalty).
 * @return double          Heuristic estimate h(conditions).
 *
 * @throws std::runtime_error for invalid mode values.
 */
inline double calcHeuristic(const SetofGCond& conditions,
                            const SetofGCond& goal_conditions,
                            int mode,
                            const Action& action) {
    int h = 0;
    if (mode == 0) {
        return 0;
    } else if (mode == 1) {
        for (const GroundedCondition& goal : goal_conditions)
            if (conditions.find(goal) == conditions.end()) ++h;
        return h;
    } else if (mode == 2) {
        for (const Condition& effect : action.get_effects())
            if (!effect.get_truth()) h += 2;
        return h;
    } else if (mode == 3) {
        for (const GroundedCondition& goal : goal_conditions)
            if (conditions.find(goal) == conditions.end()) ++h;
        for (const Condition& effect : action.get_effects())
            if (!effect.get_truth()) h += 2;
        return h;
    }
    throw std::runtime_error("Invalid heuristic mode!");
}


// --------------------------------- Backtrack ------------------------------------

/**
 * @brief Reconstruct the plan (sequence of grounded actions) from a goal node.
 *
 * Walks parent pointers back to the root, collecting each node's generating action,
 * and returns actions in forward order (from start to goal).
 *
 * @param current  Pointer to the goal node (or any node you want to reconstruct from).
 * @return std::list<GroundedAction>  Ordered plan from start to `current` (exclusive of root).
 *
 * @sideeffect Writes brief progress to std::cout.
 */
inline std::list<GroundedAction> backTrack(std::shared_ptr<Node> current)
{
    std::cout << "Backtracking..." << std::endl;
    std::list<GroundedAction> actions;
    for (auto t = current; t && t->parent; t = t->parent) {
        actions.push_front(GroundedAction(t->action.get_name(), t->action.get_args()));
    }
    std::cout << "Number of actions: " << actions.size() << std::endl;
    return actions;
}


// ---------------------------------- Planner -------------------------------------
inline std::list<GroundedAction> planner(Env* env)
{
    auto start = std::chrono::high_resolution_clock::now();
    std::list<GroundedAction> actions;

    // Start state
    auto startState = std::make_shared<Node>(
        nullptr, env->get_initial_conditions(), 0.0, 0.0, Action("", {}, {}, {}));
    if (!startState) throw std::runtime_error("Failed to create start state.");
    std::cout << "Initial state added" << std::endl;

    // OPEN list (min-heap by f)
    auto cmp = [](std::shared_ptr<Node> a, std::shared_ptr<Node> b) { return a->f > b->f; };
    std::priority_queue<
        std::shared_ptr<Node>,
        std::vector<std::shared_ptr<Node>>,
        decltype(cmp)
    > open(cmp);
    open.push(startState);
    std::cout << "Open list created and start condition added" << std::endl;

    // CLOSED set
    std::unordered_set<std::shared_ptr<Node>> closed;

    // Symbols → Permutation and combination map
    std::vector<std::string> symbols(env->get_symbols().begin(), env->get_symbols().end());
    std::sort(symbols.begin(), symbols.end());
    auto permutation_combo_map = generateGroundingMap(env, symbols);

    // Search
    while (!open.empty()) {
        auto current = open.top(); open.pop();

        if (IsGoalReached(current->conditions, env->get_goal_conditions())) {
            std::cout << "GOAL REACHED\nNumber of expanded states: " << closed.size() << std::endl;
            actions = backTrack(current);
            break;
        }

        for (const Action& action : env->get_actions()) {
            const auto it = permutation_combo_map.find(static_cast<int>(action.get_args().size()));
            if (it == permutation_combo_map.end()) continue;

            for (const auto& perm : it->second) {
                std::list<std::string> perm_list(perm.begin(), perm.end());
                Action act = updateActionWithSymbols(action, perm_list);

                if (!ArePrecondSatisfied(act, current->conditions)) continue;

                auto newConditions = applyEffect(act, current->conditions);
                if (!NotInClosedList(newConditions, closed)) continue;

                constexpr int mode = 0; // choose heuristic mode
                if (IsInOpenList(newConditions, open)) {
                    UpdateOpenList(std::make_shared<Node>(
                        current, newConditions, current->g + 1,
                        calcHeuristic(newConditions, env->get_goal_conditions(), mode, act),
                        act),
                        open);
                } else {
                    const double new_g = current->g + 1;
                    const double new_h = calcHeuristic(newConditions, env->get_goal_conditions(), mode, act);
                    open.push(std::make_shared<Node>(current, newConditions, new_g, new_h, act));
                }
            }
        }
        closed.insert(current);
    }

    // Timing
    auto end = std::chrono::high_resolution_clock::now();
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
    if (sec > 0)  std::cout << "Time taken: " << sec << " seconds\n";
    else          std::cout << "Time taken: "
                            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                            << " milliseconds\n";

    if (actions.empty()) {
        std::cout << "No plan found!" << std::endl;
        return {};
    }
    return actions;
}


int main(int argc, char *argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    // char *env_file = static_cast<char *>("example.txt");
    const char *env_file = "example.txt";
    if (argc > 1)
        env_file = argv[1];
    std::string envsDirPath = ENVS_DIR;
    char *filename = new char[envsDirPath.length() + strlen(env_file) + 2];
    strcpy(filename, envsDirPath.c_str());
    strcat(filename, "/");
    strcat(filename, env_file);

    cout << "Environment: " << filename << endl;
    Env *env = create_env(filename); //parses the environment file and creates Env object
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (const GroundedAction &gac : actions)
    {
        cout << gac << endl;
    }

    delete env;
    return 0;
}