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

typedef unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> SetofGCond;
typedef unordered_set<Condition, ConditionHasher, ConditionComparator> SetofCond;

struct Node {
    SetofGCond conditions;
    double g;
    double h;
    double f;
    shared_ptr<Node> parent;
    Action action; 

    Node(shared_ptr<Node> parent, const SetofGCond& conditions, double g, double h, const Action& action)
        : parent(parent), conditions(conditions), g(g), h(h), f(g+h), action(action) {}
};

// Function to generate all combinations of symbols
void generateCombinations(const vector<string>& symbols, int r, vector<vector<string>>& combinations, vector<string>& combination, int start) {
    if (combination.size() == r) {
        combinations.push_back(combination);
        return;
    }

    for (int i = start; i < symbols.size(); ++i) {
        combination.push_back(symbols[i]);
        generateCombinations(symbols, r, combinations, combination, i + 1);
        combination.pop_back();
    }
}

// Function to generate all permutations of a combination
void generatePermutations(const vector<string>& combination, vector<vector<string>>& permutations) {
    vector<string> perm = combination;
    sort(perm.begin(), perm.end());
    do {
        permutations.push_back(perm);
    } while (next_permutation(perm.begin(), perm.end()));
}

// Function to generate all permutations and combinations of symbols based on the number of action arguments
vector<vector<string>> generatePNC(const vector<string>& symbols, int numArgs) {
    vector<vector<string>> combinations;
    vector<string> combination;
    generateCombinations(symbols, numArgs, combinations, combination, 0);

    vector<vector<string>> permutations;
    for (const auto& comb : combinations) {
        generatePermutations(comb, permutations);
    }

    return permutations;
}

unordered_map<int, vector<vector<string>>> generatePNCMap(const Env* env, const vector<string>& symbols) {
    unordered_map<int, vector<vector<string>>> pncMap;

    for (const Action& action : env->get_actions()) {
        int numArgs = action.get_args().size();
        if (pncMap.find(numArgs) == pncMap.end()) {
            pncMap[numArgs] = generatePNC(symbols, numArgs);
        }
    }

    return pncMap;
}

void printActionArgs(const Action& action) {
    const list<string>& args = action.get_args();
    cout << "Arguments: ";
    for (const string& arg : args) {
        cout << arg << " ";
    }
    cout << endl;
}

void printPreconditions(const Action& action) {
    const auto& preconditions = action.get_preconditions();
    cout << "Preconditions: ";
    for (const auto& precond : preconditions) {
        cout << precond << " ";
    }
    cout << endl;
}

void printGConditions(const SetofGCond& conditions) {
    cout << "Ground conditions: ";
    for (const auto& condition : conditions) {
        cout << condition << " ";
    }
    cout << endl;
}

unordered_map<string, string> create_arg_map(const list<string>& old_args, const list<string>& new_args) {
    if (old_args.size() != new_args.size())
        throw runtime_error("Number of arguments do not match!");

    unordered_map<string, string> arg_map;
    auto it_old = old_args.begin();
    auto it_new = new_args.begin();
    while (it_old != old_args.end() && it_new != new_args.end()) {
        arg_map[*it_old] = *it_new;
        ++it_old;
        ++it_new;
    }
    return arg_map;
}

SetofCond update_preconditions_args(const Action& action, const list<string>& new_args) {
    unordered_map<string, string> arg_map = create_arg_map(action.get_args(), new_args);
    SetofCond updated_preconditions;

    for (const auto& precond : action.get_preconditions()) {
        list<string> precond_args = precond.get_args();
        for (auto& arg : precond_args) {
            if (arg_map.find(arg) != arg_map.end()) {
                arg = arg_map[arg];
            }
        }
        Condition updated_precond = Condition(precond.get_predicate(), precond_args, precond.get_truth());
        updated_preconditions.insert(updated_precond);
    }

    // cout << "Preconditions arguments updated!" << endl;
    return updated_preconditions;
}

SetofCond update_effects_args(const Action& action, const list<string>& new_args) {
    unordered_map<string, string> arg_map = create_arg_map(action.get_args(), new_args);
    SetofCond updated_effects;

    for (const auto& effect : action.get_effects()) {
        list<string> effect_args = effect.get_args();
        for (auto& arg : effect_args) {
            if (arg_map.find(arg) != arg_map.end()) {
                arg = arg_map[arg];
            }
        }
        Condition updated_effect = Condition(effect.get_predicate(), effect_args, effect.get_truth());
        updated_effects.insert(updated_effect);
    }

    // cout << "Effects arguments updated!" << endl;
    return updated_effects;
}

Action updateActionWithSymbols(const Action& action, const list<string>& perm_list) {

    SetofCond updated_preconditions = update_preconditions_args(action, perm_list);
    SetofCond updated_effects = update_effects_args(action, perm_list);

    return Action(action.get_name(), perm_list, updated_preconditions, updated_effects);
}

bool ArePrecondSatisfied(const Action& act, const SetofGCond& conditions)
{
    for (Condition precond : act.get_preconditions())
    {
        // printPreconditions(act);
        GroundedCondition pregc = GroundedCondition(precond.get_predicate(), precond.get_args(), precond.get_truth());
        // pregc.printArgValues();
        
        if (conditions.find(pregc) == conditions.end())
        {
            return false;
        }
    }
    return true;
}

SetofGCond applyEffect(const Action& action, const SetofGCond& conditions)
{
    SetofGCond newConditions = conditions;
    for (const Condition& effect : action.get_effects())
    {
        if (effect.get_truth())
        {
            newConditions.insert(GroundedCondition(effect.get_predicate(), effect.get_args()));
        }
        else
        {
            newConditions.erase(GroundedCondition(effect.get_predicate(), effect.get_args()));
        }
    }
    return newConditions;
}

bool IsGoalReached(const SetofGCond& conditions, const SetofGCond& goal_conditions)
{
    for (const GroundedCondition& goal : goal_conditions)
    {
        if (conditions.find(goal) == conditions.end())
        {
            return false;
        }
    }
    return true;
}

bool NotInClosedList(const SetofGCond& conditions, const unordered_set<shared_ptr<Node>>& closed)
{
    for (const shared_ptr<Node>& node : closed)
    {
        if (node->conditions == conditions)
        {
            return false;
        }
    }
    return true;
}

bool IsInOpenList(const SetofGCond& conditions, const priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, function<bool(shared_ptr<Node>, shared_ptr<Node>)>>& open) {
    auto open_copy = open; // Make a copy of the priority queue to iterate through it
    while (!open_copy.empty()) {
        shared_ptr<Node> node = open_copy.top();
        open_copy.pop();
        if (node->conditions == conditions) {
            return true;
        }
    }
    return false;
}

double calcHeuristic(const SetofGCond& conditions, const SetofGCond& goal_conditions, int mode, const Action& action)
{
    // mode 0: Dijkstra's algorithm
    // mode 1: A* algorithm (with goal conditions heuristic)
    // mode 2: A* algorithm (with empty-delete-list effects heuristic)
    // mode 3: A* algorithm (mode 1 and 2 combined)
    
    int h = 0;
    if (mode == 0){
        return h;
    } else if (mode == 1) {
        for (const GroundedCondition& goal : goal_conditions)
        {
            if (conditions.find(goal) == conditions.end())
            {
                h++;
            }
        }
        return h;
    } else if (mode == 2) {
        // Penalize actions with delete effects if the "empty-delete-list" heuristic is enabled
        for (const Condition& effect : action.get_effects()) {
            if (!effect.get_truth()) { // Check for delete effects
                h += 2;  // Increase the heuristic by a penalty factor of 1 for each delete effect
            }
        }
        return h;
    } else if (mode == 3){
        for (const GroundedCondition& goal : goal_conditions)
        {
            if (conditions.find(goal) == conditions.end())
            {
                h++;
            }
        }
        for (const Condition& effect : action.get_effects()) {
            if (!effect.get_truth()) { // Check for delete effects
                h += 2;  // Increase the heuristic by a penalty factor of 2 for each delete effect
            }
        }
        return h;
    }
    else {
        throw runtime_error("Invalid heuristic mode!");
    }
}

void UpdateOpenList(const shared_ptr<Node>& newNode, priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, function<bool(shared_ptr<Node>, shared_ptr<Node>)>>& open) {
    vector<shared_ptr<Node>> temp;
    while (!open.empty()) {
        shared_ptr<Node> node = open.top();
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
    for (const auto& node : temp) {
        open.push(node);
    }
}

list<GroundedAction> backTrack(shared_ptr<Node> current) {

    cout << "Backtracking..." << endl;
    
    list<GroundedAction> actions;
    shared_ptr<Node> temp = current;
    while (temp->parent != nullptr) {
        actions.push_front(GroundedAction(temp->action.get_name(), temp->action.get_args()));
        temp = temp->parent;
    }

    cout << "Number of actions: " << actions.size() << endl;

    return actions;
}

list<GroundedAction> planner(Env *env)
{
    //////////////////////////////////////////
    ///// TODO: INSERT YOUR PLANNER HERE /////
    //////////////////////////////////////////

    auto start = chrono::high_resolution_clock::now(); 
    list<GroundedAction> actions;
    
    // Create a state
    auto startState = make_shared<Node>(nullptr, env->get_initial_conditions(), 0, 0, Action("", {}, {}, {}));
    if (!startState) {
        throw runtime_error("Failed to create start state.");
    } else {
        cout << "Initial state added" << endl;
    }

    // Create a priority queue
    priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, function<bool(shared_ptr<Node>, shared_ptr<Node>)>> open([](shared_ptr<Node> a, shared_ptr<Node> b) { return a->f > b->f; });
    open.push(startState);
    if (open.empty()) {
        throw runtime_error("Failed to create open list.");
    } else {
        cout << "Open list created and start condition added" << endl;
    }

    // Create a closed list
    unordered_set<shared_ptr<Node>> closed;

    vector<string> symbols(env->get_symbols().begin(), env->get_symbols().end());
    // Sort symbols to ensure we start with the lexicographically smallest permutation
    sort(symbols.begin(), symbols.end()); 

    auto pncMap = generatePNCMap(env, symbols);
    
    while(!open.empty()){

        shared_ptr<Node> current = open.top();
        open.pop();
        // cout << "Current state: " << current->conditions.size() << endl;

        if (IsGoalReached(current->conditions, env->get_goal_conditions())){
            cout << "GOAL REACHED" << endl;
            cout << "Number of expanded states: " << closed.size() << endl;
            actions =  backTrack(current);
            break;
        }

        for (const Action& action : env->get_actions()){
            // cout << "Action: " << action.get_name() << endl;

            for (const auto& perm : pncMap[action.get_args().size()]) {

                // cout << "Inside PNC loop" << endl;
                
                list<string> perm_list(perm.begin(), perm.end());
                Action act = updateActionWithSymbols(action, perm_list);  

                if (ArePrecondSatisfied(act, current->conditions)){
                    // cout << "Preconditions satisfied" << endl;

                    auto newConditions = applyEffect(act, current->conditions);
                    // cout << "Effects applied" << endl;

                    if (NotInClosedList(newConditions, closed)){

                        // mode 0: Dijkstra's algorithm
                        // mode 1: A* algorithm (with goal conditions heuristic)
                        // mode 2: A* algorithm (with empty-delete-list effects heuristic)
                        // mode 3: A* algorithm (mode 1 and 2 combined)
                        int mode = 3;

                        if (IsInOpenList(newConditions, open)){
                            UpdateOpenList(make_shared<Node>(current, newConditions, current->g + 1, calcHeuristic(newConditions, env->get_goal_conditions(), mode, act), act), open);
                        } 
                        else {
                            auto new_g = current->g + 1;
                            auto new_h = calcHeuristic(newConditions, env->get_goal_conditions(), mode, act);
                            auto newNode = make_shared<Node>(current, newConditions, new_g, new_h, act);
                            open.push(newNode);
                            // cout << "New state added" << endl;
                            // printGConditions(newNode->conditions);
                        }
                    }
                }
            }
        }
        closed.insert(current);
    }
    // actions.push_back(GroundedAction("MoveToTable", {"A", "B"}));
    // actions.push_back(GroundedAction("Move", {"C", "Table", "A"}));
    // actions.push_back(GroundedAction("Move", {"B", "Table", "C"}));
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_sec = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
    if (duration_sec > 0){
        cout << "Time taken: " << duration_sec << " seconds" << endl;
    }
    else {
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        cout << "Time taken: " << duration_ms << " milliseconds" << endl;
    }


    if (actions.empty()){
        cout << "No plan found!" << endl;
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