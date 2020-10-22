
#################################### FG classes ####################################
# attribute for different types of factors/variables
import copy

class Shape:
    def __init__(self, shape_type, color, loc, size):
        self.shape_type = shape_type
        self.color = color
        self.loc = loc
        self.size = size

    def printLatex(self, f):
        f.write("\\draw [{}] ({}, {}) {} ({});\n".format(self.color, self.loc[0], self.loc[1], self.shape_type, self.size))

class Color:
    def __init__(self, name, rgb):
        self.name = name
        self.rgb = rgb

    def printLatex(self, f):
        f.write("\definecolor {{{}}} {{rgb}} {{ {},{},{} }};\n".format(self.name, self.rgb[0], self.rgb[1], self.rgb[2]))


class Legend:
    def __init__(self, attribute, content, relative_loc, text_scale, name=None):
        self.attribute = attribute
        self.content = content
        self.relative_loc = relative_loc
        self.text_scale = text_scale
        self.name = name

class Legends:
    def __init__(self, legends, loc):
        self.legends = list(legends)
        self.loc = loc
        self.legend_count = 0
    
    def printLatex(self, f):
        for legend in self.legends:
            if legend.name is None:
                self.legend_count += 1
                legend.name = "legend" + str(self.legend_count)

        f.write("\\coordinate (legend) at ({}, {});\n".format(self.loc[0], self.loc[1]))
        for legend in self.legends:
            f.write("\\coordinate ({}) at ({}, {});\n".format(legend.name, self.loc[0]+legend.relative_loc[0], self.loc[1]+legend.relative_loc[1]))
        for legend in self.legends:
            f.write("\\node[{}, scale={}, fill={}] at ({}) {{}};\n".format(legend.attribute.shape_type, legend.attribute.size, legend.attribute.color, legend.name))
            f.write("\\node[draw=none, scale={}, anchor=west, inner sep=6pt] at ({}) {{{}}};\n".format(legend.text_scale, legend.name, legend.content))

class Attribute:
    def __init__(self, color="white", size = 0.1, shape_type="circle"):
        self.color = color
        self.size = size
        self.shape_type = shape_type

class Variable:
    def __init__(self, attribute, loc, name=None, symbol=None):
        self.attribute = attribute
        self.loc = list(loc)
        self.name = name
        self.symbol = symbol

class Factor:
    def __init__(self, attribute, variables, loc=None, name=None, eqn=None, eqn_loc=[]):
        self.attribute = attribute
        self.variables = list(variables)
        if loc is not None:
            self.loc = list(loc)
        else:
            average_loc = [0, 0]
            for variable in variables:
                average_loc[0] += variable.loc[0]
                average_loc[1] += variable.loc[1]
            average_loc[0] /= len(variables)
            average_loc[1] /= len(variables)
            self.loc = average_loc
        self.name = name
        self.eqn = eqn
        self.eqn_loc = eqn_loc

class Conditional:
    def __init__ (self, attribute, variable, parents, name=None, eqn=None, eqn_loc=[]):
        self.attribute = attribute
        self.variable = variable
        self.parents = list(parents)
        self.name = name
        self.eqn = eqn
        self.eqn_loc = eqn_loc

class Graph:
    def __init__(self):
        self.variables = {}
        self.factors = {}
        self.conditionals = {}
        self.shapes = []
        self.colors = []
        self.v_count = 0
        self.f_count = 0

    def addVariable(self, variable):
        # TODO: this can be done more efficiently
        to_add = True
        for v in self.variables.values():
            if variable is v:
                to_add = False
                break
        if to_add:
            if variable.name is None:
                self.v_count += 1
                variable.name = "v"+str(self.v_count)
            self.variables[variable.name] = variable

    def addFactor(self, factor):
        if factor.name is None:
            self.f_count += 1
            factor.name = "f"+str(self.f_count)
        self.factors[factor.name] = factor
        for variable in factor.variables:
            self.addVariable(variable)
    
    def addConditional(self, conditional):
        if conditional.name is None:
            self.f_count += 1
            factor.name = "f"+str(self.f_count)
        self.conditionals[conditional.name] = conditional
        self.addVariable(conditional.variable)
        for variable in conditional.parents:
            self.addVariable(variable)

    def printLatex(self, f):
        # set locations
        for variable in self.variables.values():
            f.write("\\coordinate ({}) at ({}, {});\n".format(variable.name+"_coord", variable.loc[0], variable.loc[1]))

        for factor in self.factors.values():
            f.write("\\coordinate ({}) at ({}, {});\n".format(factor.name+"_coord", factor.loc[0], factor.loc[1]))

        for factor in self.factors.values():
            for variable in factor.variables:
                f.write("\\path[draw={}, line width=0.3pt] ({}) -- ({});\n".format(factor.attribute.color, variable.name+"_coord", factor.name+"_coord"))

        for variable in self.variables.values():
            content = ""
            if variable.symbol is not None:
                content = "${}$".format(variable.symbol)
            f.write("\\node[scale={}, fill={}][circle, inner sep=2.8pt, draw, very thin] at ({}) ({}) {{{}}};\n".format(variable.attribute.size, variable.attribute.color, variable.name+"_coord", variable.name, content))

        for factor in self.factors.values():
            f.write("\\node[{}, scale={}, fill={}] at ({}) ({}) {{}};\n".format(factor.attribute.shape_type, factor.attribute.size, factor.attribute.color, factor.name+"_coord", factor.name))

        for factor in self.factors.values():
            if factor.eqn is not None:
                f.write("\\node[rectangle callout, draw=none, inner sep=2.8pt, rounded corners=1pt, fill = gray!20, callout absolute pointer={{({})}}, scale = 0.25] at ({},{}) {{${}$}};\n".format(factor.name+"_coord", factor.eqn_loc[0], factor.eqn_loc[1], factor.eqn)) 

        for conditional in self.conditionals.values():
            variable = conditional.variable
            for parent in conditional.parents:
                f.write("\\path[->, >=stealth, draw={}, line width=0.3pt] ({}) -- ({});\n".format(conditional.attribute.color, parent.name, variable.name))

#################################### locations ####################################
def gridLocation(x, y):
    return [x*1.2, y]
def fc(color="black", size=0.3):
    return Attribute(color=color, size=size, shape_type="rectangle")  # constrained
def fo(color="black", size=0.3):
    return Attribute(color=color, size=size, shape_type="circle")  # objective
def cond(color="gray!50"):
    return Attribute(color=color, size=0.1) # conditional


v_size = 0.5
def vx():
    return Attribute(color="white", size=v_size)
def vu():
    return Attribute(color="white", size=v_size)

def getFG(constrained=False):
    x0 = Variable(vx(), gridLocation(0, 0), symbol="x_0", name="x0")
    x1 = Variable(vx(), gridLocation(1, 0), symbol="x_1", name="x1")
    x2 = Variable(vx(), gridLocation(2, 0), symbol="x_2", name="x2")
    u0 = Variable(vu(), gridLocation(0.5, -1), symbol="u_0", name="u0")
    u1 = Variable(vu(), gridLocation(1.5, -1), symbol="u_1", name="u1")


    graph = Graph()
    graph.addFactor(Factor(fo(), [x0], loc=gridLocation(0.4, 0.2), eqn="x_0^TQx_0", eqn_loc=gridLocation(0.4, 0.5), name="soft_x0"))
    graph.addFactor(Factor(fo(), [x1], loc=gridLocation(1.4, 0.2), eqn="x_1^TQx_1", eqn_loc=gridLocation(1.4, 0.5), name="soft_x1"))
    graph.addFactor(Factor(fo(), [x2], loc=gridLocation(2.4, 0.2), eqn="x_2^TQx_2", eqn_loc=gridLocation(2.4, 0.5), name="soft_x2"))
    graph.addFactor(Factor(fo(), [u0], loc=gridLocation(0.1, -1.2), eqn="u_0^TRu_0", eqn_loc=gridLocation(0.1, -1.5), name="soft_u0"))
    graph.addFactor(Factor(fo(), [u1], loc=gridLocation(1.1, -1.2), eqn="u_1^TRu_1", eqn_loc=gridLocation(1.1, -1.5), name="soft_u1"))
    graph.addFactor(Factor(fc(), [x0, u0, x1], eqn="x_1=A_0x_0+B_0u_0", eqn_loc=gridLocation(0.9, -0.6), name="dynamics01"))
    graph.addFactor(Factor(fc(), [x1, u1, x2], eqn="x_2=A_1x_1+B_1u_1", eqn_loc=gridLocation(1.9, -0.6), name="dynamics12"))

    if constrained:
        graph.factors["dynamics01"].eqn_loc = gridLocation(0.5, 0)
        graph.factors["dynamics12"].eqn_loc = gridLocation(1.5, 0)
        graph.addFactor(Factor(fc(), [x0, u0], eqn="C_0x_0+D_0u_0+r_0=0", eqn_loc=gridLocation(-0.1, -0.7), name="constrain0"))
        graph.addFactor(Factor(fc(), [x1, u1], eqn="C_1x_1+D_1u_1+r_1=0", eqn_loc=gridLocation(0.9, -0.7), name="constrain1"))
        graph.addFactor(Factor(fc(), [x2], loc=gridLocation(2.25, -0.5), eqn="C_2x_2+r_2=0", eqn_loc=gridLocation(2.25, -0.7), name="constrain2"))
    return graph

def getLegends():
    legend_list = [Legend(fo(), "Gaussian Factor", [0.1, 0.22], 0.3), Legend(fc(), "Constraint Factor", [0.1, 0.08], 0.3)]
    legends = Legends(legend_list, gridLocation(2, -1.5))
    return legends

def exportTikz(file_path, graph, legends=None, colors=None):
    with open(file_path, "w") as f:
        f.write("\\begin{tikzpicture}\n")
        if colors is not None:
            for color in colors:
                color.printLatex(f)
        graph.printLatex(f)
        if legends is not None:
            legends.printLatex(f)
        f.write("\\end{tikzpicture}\n")

def lqr():
    graph = getFG(constrained=False)
    legends = getLegends()
    exportTikz("tikz/LQR_fg.tikz", graph, legends)

def constrainedLqr():
    graph = getFG(constrained=True)
    legends = getLegends()
    exportTikz("tikz/LQR_fg.tikz", graph, legends)

def lqrElimination():
    colors = [Color("dgreen", [0, 0.5, 0])]

    graph = getFG(constrained=False)
    for factor in graph.factors.values():
        factor.eqn = None
        factor.attribute.size = 0.5
    for variable in graph.variables.values():
        variable.attribute.size = 0.7

    graph.factors["soft_x2"].attribute.color = "red"
    graph.factors["dynamics12"].attribute.color = "red"
    exportTikz("tikz/lqr_elim_x2_1.tikz", graph, colors=colors)


    del graph.factors["soft_x2"]
    del graph.factors["dynamics12"]
    graph.addConditional(Conditional(cond("dgreen"), graph.variables["x2"], [graph.variables["x1"], graph.variables["u1"]], name="cond_x2"))
    graph.addFactor(Factor(fo("blue", 0.5), [graph.variables["x1"], graph.variables["u1"]], name="marg_x2"))
    exportTikz("tikz/lqr_elim_x2_2.tikz", graph, colors=colors)

    graph.conditionals["cond_x2"].attribute.color = "gray!50"
    graph.factors["marg_x2"].attribute.color = "red"
    graph.factors["soft_u1"].attribute.color = "red"
    exportTikz("tikz/lqr_elim_u1_1.tikz", graph, colors=colors)

    del graph.factors["marg_x2"]
    del graph.factors["soft_u1"]
    graph.addConditional(Conditional(cond("dgreen"), graph.variables["u1"], [graph.variables["x1"]], name="cond_u1"))
    graph.addFactor(Factor(fo("blue", 0.5), [graph.variables["x1"]], loc=gridLocation(1, 0.4), name="marg_u1"))
    exportTikz("tikz/lqr_elim_u1_2.tikz", graph, colors=colors)

def constrainedLqrElimination():
    colors = [Color("dgreen", [0, 0.5, 0])]
    graph = getFG(constrained=True)
    factor_size = 0.5
    variable_size = 0.7
    for factor in graph.factors.values():
        factor.eqn = None
        factor.attribute.size = factor_size
    for variable in graph.variables.values():
        variable.attribute.size = variable_size

    graph.factors["soft_x2"].attribute.color = "red"
    graph.factors["dynamics12"].attribute.color = "red"
    graph.factors["constrain2"].attribute.color = "red"
    exportTikz("tikz/eq_lqr_elim_x2_1.tikz", graph, colors=colors)

    del graph.factors["soft_x2"]
    del graph.factors["dynamics12"]
    del graph.factors["constrain2"]
    graph.addConditional(Conditional(cond("dgreen"), graph.variables["x2"], [graph.variables["x1"], graph.variables["u1"]], name="cond_x2"))
    graph.addFactor(Factor(fc("blue", factor_size), [graph.variables["x1"], graph.variables["u1"]], loc=gridLocation(1.5, -0.4), name="hard_marg_x2"))
    graph.addFactor(Factor(fo("blue", factor_size), [graph.variables["x1"], graph.variables["u1"]], loc=gridLocation(1,-0.6),  name="soft_marg_x2"))
    exportTikz("tikz/eq_lqr_elim_x2_2.tikz", graph, colors=colors)

    graph.conditionals["cond_x2"].attribute.color = "gray!50"
    graph.factors["hard_marg_x2"].attribute.color = "red"
    graph.factors["soft_marg_x2"].attribute.color = "red"
    graph.factors["constrain1"].attribute.color = "red"
    graph.factors["soft_u1"].attribute.color = "red"
    exportTikz("tikz/eq_lqr_elim_u1_1.tikz", graph, colors=colors)

    del graph.factors["hard_marg_x2"]
    del graph.factors["soft_marg_x2"]
    del graph.factors["soft_u1"]
    del graph.factors["constrain1"]
    graph.addConditional(Conditional(cond("dgreen"), graph.variables["u1"], [graph.variables["x1"]], name="cond_u1"))
    graph.addFactor(Factor(fo("blue", 0.5), [graph.variables["x1"]], loc=gridLocation(1, 0.4), name="soft_marg_u1"))
    graph.addFactor(Factor(fc("blue", 0.5), [graph.variables["x1"]], loc=gridLocation(0.8, 0.3), name="hard_marg_u1"))
    exportTikz("tikz/eq_lqr_elim_u1_2.tikz", graph, colors=colors)

#################################### main function ####################################
def main():
    # lqr()
    lqrElimination()
    constrainedLqrElimination()

if __name__ == "__main__":
    main()