import sphinx.locale
import docutils.statemachine
sphinx.locale.admonitionlabels['embed'] = u'' #u'Default Embedded Stage'
sphinx.locale.admonitionlabels['plugin'] = u''# u'Non-default Dynamic Plugin Stage'
sphinx.locale.admonitionlabels['streamable'] = u''# u'Streamable Stage'

def setup(app):
    app.add_node(embed,
                 html=(visit_embed_node, depart_node),
                 latex=(visit_admonition, depart_node),
                 text=(visit_admonition, depart_node))

    app.add_node(plugin,
                 html=(visit_plugin_node, depart_node),
                 latex=(visit_admonition, depart_node),
                 text=(visit_admonition, depart_node))
    app.add_node(streamable,
                 html=(visit_streamable_node, depart_node),
                 latex=(visit_admonition, depart_node),
                 text=(visit_admonition, depart_node))
    app.add_directive('embed', EmbedDirective)
    app.add_directive('plugin', PluginDirective)
    app.add_directive('streamable', StreamableDirective)
    app.connect('env-purge-doc', purge_embeds)
    return {'version': '0.1'}   # identifies the version of our extension

from docutils import nodes

class embed(nodes.Admonition, nodes.Element):
    pass

class plugin(nodes.Admonition, nodes.Element):
    pass

class streamable(nodes.Admonition, nodes.Element):
    pass

def visit_admonition(self, node):
    self.visit_admonition(node)

def visit_embed_node(self, node):
    self.body.append(self.starttag(
            node, 'div', CLASS=('admonition embed')))
#    self.set_first_last(node)

def visit_plugin_node(self, node):
    self.body.append(self.starttag(
            node, 'div', CLASS=('admonition plugin')))
#    self.set_first_last(node)

def visit_streamable_node(self, node):
    self.body.append(self.starttag(
            node, 'div', CLASS=('admonition streamable')))
#    self.set_first_last(node)

def depart_node(self, node):
    self.depart_admonition(node)


from docutils.parsers.rst import Directive


from sphinx.locale import _

class EmbedDirective(Directive):

    # this enables content in the directive
    has_content = True

    def run(self):
        env = self.state.document.settings.env

        targetid = "embed-%d" % env.new_serialno('embed')
        targetnode = nodes.target('', '', ids=[targetid])

#        self.content = 'This stage is enabled by default'
        self.content = docutils.statemachine.StringList(['This stage is enabled by default'])
        embed_node = embed('\n'.join(self.content))
        embed_node += nodes.title(_('Default Embedded Stage'), _('Default Embedded Stage '))
        self.state.nested_parse(self.content, self.content_offset, embed_node)

        if not hasattr(env, 'embed_all_embeds'):
            env.embed_all_embeds = []
        env.embed_all_embeds.append({
            'docname': env.docname,
            'lineno': self.lineno,
            'embed': embed_node.deepcopy(),
            'target': targetnode,
        })

        return [targetnode, embed_node]

class PluginDirective(Directive):

    # this enables content in the directive
    has_content = True

    def run(self):
        env = self.state.document.settings.env

        targetid = "plugin-%d" % env.new_serialno('plugin')
        targetnode = nodes.target('', '', ids=[targetid])

#        self.content = 'This stage requires a dynamic plugin to operate'
        self.content = docutils.statemachine.StringList(['This stage requires a dynamic plugin to operate'])

        plugin_node = plugin('\n'.join(self.content))
        plugin_node += nodes.title(_('Dynamic Plugin'), _('Dynamic Plugin'))
        self.state.nested_parse(self.content, self.content_offset, plugin_node)

        if not hasattr(env, 'plugin_all_plugins'):
            env.plugin_all_plugins = []
        env.plugin_all_plugins.append({
            'docname': env.docname,
            'lineno': self.lineno,
            'plugin': plugin_node.deepcopy(),
            'target': targetnode,
        })

        return [targetnode, plugin_node]

class StreamableDirective(Directive):

    # this enables content in the directive
    has_content = True

    def run(self):
        env = self.state.document.settings.env

        targetid = "streamable-%d" % env.new_serialno('streamable')
        targetnode = nodes.target('', '', ids=[targetid])

#        self.content = 'This stage supports streaming operations'
        self.content = docutils.statemachine.StringList(['This stage supports streaming operations'])
        streamable_node = streamable('\n'.join(self.content))
        streamable_node += nodes.title(_('Streamable Stage'), _('Streamable Stage'))
        self.state.nested_parse(self.content, self.content_offset, streamable_node)

        if not hasattr(env, 'streamable_all_streamable'):
            env.streamable_all_streamable = []
        env.streamable_all_streamable.append({
            'docname': env.docname,
            'lineno': self.lineno,
            'plugin': streamable_node.deepcopy(),
            'target': targetnode,
        })

        return [targetnode, streamable_node]

def purge_embeds(app, env, docname):
    if not hasattr(env, 'embed_all_embeds'):
        return
    env.embed_all_embeds = [embed for embed in env.embed_all_embeds
                          if embed['docname'] != docname]

    if not hasattr(env, 'plugin_all_plugins'):
        return
    env.plugin_all_plugins = [embed for embed in env.plugin_all_plugins
                          if embed['docname'] != docname]

    if not hasattr(env, 'streamable_all_streamable'):
        return
    env.streamable_all_streamable = [embed for embed in env.streamable_all_streamable
                          if embed['docname'] != docname]
